#ifndef BEHAVIOR_MANAGER_BEHAVIOR_BASE_H
#define BEHAVIOR_MANAGER_BEHAVIOR_BASE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <algorithm>
#include <cctype>
#include <cmath>

// 引入 pluginlib，和抽象接口头
#include <pluginlib/class_loader.h>
#include "variable_extractor/VariableExtractorInterface.hpp"

namespace behavior_manager {

/**
 * @brief 行为的状态枚举
 */
enum class BehaviorState {
    IDLE,       // 行为未激活或已停用
    ACTIVE,     // 行为已激活（条件满足，准备运行）
    RUNNING,    // 行为正在执行 execute()
    COMPLETED,  // 行为正常完成
    FAILED      // 行为执行失败
};

/**
 * @brief 将字符串转换为 BehaviorState
 * 
 * - 参数 s 支持大小写混合，例如 “idle”/“IDLE”/“Idle” 都能正确匹配
 * - 仅支持 IDLE、ACTIVE、RUNNING、COMPLETED、FAILED，其他一律映射到 IDLE
 */
inline BehaviorState stringToState(const std::string& s) {
    std::string u = s;
    std::transform(u.begin(), u.end(), u.begin(), ::toupper);
    if (u == "IDLE")      return BehaviorState::IDLE;
    if (u == "ACTIVE")    return BehaviorState::ACTIVE;
    if (u == "RUNNING")   return BehaviorState::RUNNING;
    if (u == "COMPLETED") return BehaviorState::COMPLETED;
    if (u == "FAILED")    return BehaviorState::FAILED;
    // 默认返回 IDLE，避免未知字符串导致崩溃
    return BehaviorState::IDLE;
}

/**
 * @brief 变量条件结构体：例如 SPEED <= 0.5
 * 
 * - var_name：变量名称（如 "SPEED"/"DEPTH"/"HEADING"/"PITCH"/"ROLL"）
 * - op：运算符，支持 "<", "<=", ">", ">=", "==" 五种
 * - threshold：阈值，例如 0.5、3.0 等
 */
struct VarCondition {
    std::string var_name;
    std::string op;
    double      threshold;
};

/**
 * @brief BehaviorBase：所有“不依赖 ActionLib”的行为共用的基类
 * 
 * 核心功能：
 *   1. 私有命名空间与回调队列：每个行为单独创建 ros::NodeHandle(namespace = instance_name_)
 *      并使用独立的 CallbackQueue + AsyncSpinner，以保证多个行为同时存在时互不影响。
 *   2. 通用参数读取：
 *      - priority（int，值越小优先级越高）
 *      - pwt（double，同一优先级下的权重，值越大优先程度越强）
 *      - duration（运行超时时长，单位秒；<=0 表示不检查）
 *      - duration_idle_decay（当行为处于 IDLE 时，是否继续累积 duration 计时）
 *      - timeout_fails（超时后是否标记为 FAILED；否则标记为 COMPLETED）
 *      - enable_debug（打开后会打印 ROS_DEBUG 日志）
 *      - vehicle_name（车辆名称，用于拼接全局 Topic 前缀）
 *      - variable_extractor_plugin（要加载的插件名称，例如 "behavior_manager/DefaultVariableExtractor"）
 *      - conditions（从 YAML 中读取的“启动条件”字符串列表）
 *   3. 通用条件解析与订阅：
 *      - <STATE>:<behavior_name> 类型：订阅 /<vehicle_name>/behavior_states/<other> 话题，
 *        且记录其他行为最新状态
 *      - <VARIABLE>:<op><value> 类型：通过 pluginlib 加载的 VariableExtractorInterface，从对应 Topic
 *        提取变量并更新到 variable_values_
 *   4. 三类计时器：
 *      - duration_timer_：记录行为处于 RUNNING 状态（或 IDLE+idle_decay=true 时）的累计时间
 *                        用于检测“运行超时”
 *      - active_timer_：记录行为从 ACTIVE 变到 IDLE 之前所消耗的时间（可供统计/调度使用）
 *      - idle_timer_：记录行为处于 IDLE 状态的累计时长（可用于判断“空闲”过久）
 *   5. 行为生命周期钩子：
 *      - onActivate()：行为进入 ACTIVE 状态时调用
 *      - onDeactivate(): 行为进入 IDLE 状态时调用
 *      - onComplete():   行为正常完成时调用
 *      - onFailure():    行为失败时调用
 *      - onTimeout():    运行超时（duration >= limit）时调用，默认依 timeout_fails_ 标记状态
 *   6. 状态发布：
 *      - 通过 /<vehicle_name>/behavior_states/<instance_name> 话题发布状态字符串
 */
class BehaviorBase {
public:
    BehaviorBase(const std::string& instance_name);
    virtual ~BehaviorBase();

    /**
     * @brief 第一阶段：初始化
     *  - 在 parent_nh（通常由 BehaviorManager 传入）下创建私有命名空间 nh_
     *  - 设置私有 CallbackQueue 和一线程 AsyncSpinner
     *  - 读取通用参数：priority, pwt, duration, duration_idle_decay, timeout_fails,
     *    enable_debug, vehicle_name, variable_extractor_plugin, conditions
     *  - 自动创建并发布状态 Publisher：/<vehicle_name>/behavior_states/<instance_name>
     *  - 创建三类定时器：
     *       * duration_timer_（10Hz，累计 RUNNING 时间，默认不 autostart）
     *       * active_timer_（10Hz，累计 ACTIVE 时间，默认不 autostart)
     *       * idle_timer_  （10Hz，累计 IDLE 时间, 默认不 autostart)
     *  - 解析并订阅<STATE>:<behavior_name> 条件
     *  - 通过 pluginlib 动态加载 VariableExtractor 插件，并调用 subscribe() 订阅 <VARIABLE>:<op><value> 条件
     *
     *  在 roslaunch 中可通过参数 <namespace>/variable_extractor_plugin 来指定要加载的插件名称。
     */
    virtual void initialize(ros::NodeHandle& parent_nh);

    /**
     * @brief 第二阶段：激活
     *  - 将内部状态设为 ACTIVE
     *  - 清零“三类计时器”累计时间
     *  - 启动 active_timer_（记录 ACTIVE 时长）和 spinner_
     *  - 如果配置了 duration_limit_ > 0，则同时启动 duration_timer_
     *  - 发布一次 "ACTIVE" 到 /<vehicle_name>/behavior_states/<instance_name>
     *  - 调用子类的 onActivate()
     */
    virtual void activate();

    /**
     * @brief run()：由 BehaviorManager 在主循环中调用
     *  - 仅当 state_ == ACTIVE 或 RUNNING 时才会执行
     *  - 切到 RUNNING 并调用子类的 execute()
     *  - 如果仍然是 RUNNING，就检查 duration 超时，若超时则调用 onTimeout()
     *  - 如果子类把 state_ 设为 COMPLETED/FAILED，则调用对应钩子
     */
    void run();

    /**
     * @brief 第四阶段：停用（切回 IDLE）
     *  - 停止 active_timer_（ACTIVE 计时）和 duration_timer_（如果在跑）
     *  - 启动 idle_timer_（记录 IDLE 时长）
     *  - 将状态置为 IDLE 并发布一次 “IDLE”
     *  - 调用子类的 onDeactivate()
     */
    virtual void deactivate();

    /**
     * @brief 第五阶段：清理 / 完成
     *  - 由管理器主动调用
     *  - 停止所有计时器与 spinner_
     *  - 根据当前状态发布最后一次 “COMPLETED”/“FAILED”
     *  - 调用对应钩子
     */
    virtual void cleanup();

    /** 获取当前状态 */
    BehaviorState getState() const { return state_; }

    /** 获取优先级等级（值越小优先级越高） */
    int getPriority() const { return priority_; }

    /** 获取同一优先级下的权重 pwt （值越大偏好度越高） */
    double getPWT() const { return pwt_; }

    /** 派生类必须实现：返回唯一实例名称，与 YAML 中配置的 name 一致 */
    virtual std::string getName() const {
        return instance_name_;
    }

    /**
     * @brief 派生类必须实现：具体执行逻辑
     *   - 当完成条件满足时，应显式调用 setCompleted()
     *   - 当发生错误时，应显式调用 setFailed()
     */
    virtual void execute() = 0;

    /**
     * @brief 子类可选重载：自定义条件判断逻辑
     *  - 如果派生类重写了 checkConditionImpl()，则使用子类逻辑
     *  - 否则基类自动检查 YAML 中解析出的 state_conditions_ 与 var_conditions_
     */
    virtual bool checkCondition() const;

protected:
    // ====== 生命周期钩子 ======
    // 派生类可选重载：行为从 IDLE → ACTIVE 时调用
    virtual void onActivate();

    // 派生类可选重载：行为从 ACTIVE → IDLE 时调用
    virtual void onDeactivate();

    // 派生类可选重载：行为正常完成时调用
    virtual void onComplete();

    // 派生类可选重载：行为失败时调用
    virtual void onFailure();
    /**
     * @brief duration 超时调用，默认根据 timeout_fails_ 来标记 COMPLETED/FAILED。
     *  子类可以重载本方法重新定义超时行为，但应保持发布状态与调用 onComplete()/onFailure()。
     */
    virtual void onTimeout();

    // ====== 私有计时器回调 ======
    virtual void onDurationTimer(const ros::TimerEvent&);
    virtual void onActiveTimer(const ros::TimerEvent&);
    virtual void onIdleTimer(const ros::TimerEvent&);
    virtual void onDurationReset(const std_msgs::Empty::ConstPtr&);

    // ====== 其他行为状态或变量更新回调 ======
    virtual void onOtherBehaviorState(const std_msgs::String::ConstPtr& msg,
                                      const std::string& other_name);
    virtual void onVariableValue(const std_msgs::Float32::ConstPtr& msg,
                                 const std::string& var_name);

    // ====== 子类可调用辅助方法 ======
    void setCompleted();
    void setFailed();
    void resetActiveTime();
    void resetIdleTime();
    void startActiveTimer();
    void stopActiveTimer();
    void startIdleTimer();
    void stopIdleTimer();

    /** 子类重写以实现完全自定义的条件判断 */
    virtual bool checkConditionImpl() const { return true; }

private:
    // ====== 解析 YAML 中的 conditions（“STATE:name” / “VAR:op+value”）到内部结构 ======
    void parseConditions(const std::vector<std::string>& cond_list);

    /** 去除字符串两端空白 */
    static void trim(std::string& s);

    /** 发布当前状态到 /<vehicle_name>/behavior_states/<instance_name> */
    void publishState(const std::string& s);

private:
    // === ROS 相关 ===
    ros::NodeHandle               nh_parent_;   // 父 NodeHandle
    ros::NodeHandle               nh_;          // 私有 NodeHandle(namespace=instance_name_)
    ros::CallbackQueue            cb_queue_;    // 私有 CallbackQueue
    std::unique_ptr<ros::AsyncSpinner> spinner_;

    // === 实例信息 ===
    std::string instance_name_;    // 行为唯一实例名称
    std::string vehicle_name_;     // 车辆名称，从 YAML 最顶层读取

    // === 插件加载器 ===
    pluginlib::ClassLoader<variable_extractor::VariableExtractorInterface> * extractor_loader_{nullptr};
    variable_extractor::VariableExtractorInterface * extractor_instance_{nullptr};

    // === 优先级 / 权重 ===
    int    priority_{1};   // 等级，值越小优先级越高 (0 为最高)
    double pwt_{1.0};      // 同级权重，值越大偏好度越高

    // === 三类计时器及累计值 ===
    double      duration_limit_{0.0};       // 最大运行时长 (秒)，<=0 表示不检查
    bool        duration_idle_decay_{false};// IDLE 时是否继续累加 duration
    bool        timeout_fails_{false};      // 超时后是否标记为 FAILED，否则标记为 COMPLETED
    double      elapsed_duration_{0.0};     // RUNNING / (IDLE+idle_decay) 下累计秒数
    ros::Timer  duration_timer_;            // 10Hz，累计 RUNNING 时间

    double      elapsed_active_{0.0};       // ACTIVE 下累计秒数
    ros::Timer  active_timer_;              // 10Hz，累计 ACTIVE 时间

    double      elapsed_idle_{0.0};         // IDLE 下累计秒数
    ros::Timer  idle_timer_;                // 10Hz，累计 IDLE 时间

    ros::Subscriber sub_duration_reset_;     // 复位 duration 的 Subscriber
    ros::Publisher  pub_duration_status_;    // 发布 duration 累计值

    // === 通用条件配置 ===
    struct StateCondition { std::string behavior_name; BehaviorState desired_state; };
    std::vector<StateCondition>     state_conditions_;  // “STATE:otherBehavior” 列表
    std::vector<VarCondition>       var_conditions_;    // “VAR:op+value” 列表

    std::map<std::string, BehaviorState> other_behavior_states_; // 存储其他行为最新状态
    std::map<std::string, double>        variable_values_;      // 存储变量最新值

    std::set<std::string>           subscribed_behavior_state_topics_; // 已订阅的行为状态集合
    std::set<std::string>           subscribed_variable_topics_;       // 已订阅的变量集合

    std::vector<ros::Subscriber>    other_state_subs_;    // 保存行为状态订阅器，防析构
    std::vector<ros::Subscriber>    variable_subs_;       // 保存变量订阅器，防析构

    // === 状态发布 ===
    std::string     state_topic_;    // "/<vehicle_name>/behavior_states/<instance_name>"
    ros::Publisher  pub_state_;

    // === 当前状态 ===
    BehaviorState   state_{BehaviorState::IDLE};

    // === 检测子类是否 override checkConditionImpl() ===
    bool hasOverrideCheckCondition_{false};

    // === 调试开关 ===
    bool debug_{false};
};

}  // namespace behavior_manager

#endif  // BEHAVIOR_MANAGER_BEHAVIOR_BASE_H
