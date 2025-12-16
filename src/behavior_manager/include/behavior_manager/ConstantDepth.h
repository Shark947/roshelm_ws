#ifndef SOLO_BEHAVIORS_CONSTANT_DEPTH_H
#define SOLO_BEHAVIORS_CONSTANT_DEPTH_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include <behavior_manager/BehaviorWithAction.h>
#include <behavior_manager/DepthBehaviorAction.h>  // 由 DepthBehavior.action 生成
#include "variable_extractor/VariableExtractorInterface.hpp"    // 用于提取 “DEPTH” 变量

namespace solo_behaviors
{

/**
 * @brief ConstantDepth 行为：在指定深度附近保持
 *
 * 继承自 BehaviorWithAction<DepthBehaviorAction>，使用同一个通用的 DepthBehavior.action：
 *  - Goal:
 *      * target_depth (float64)：目标深度（米）
 *      * publish_mismatch (bool)：是否发布深度误差
 *  - Feedback:
 *      * current_depth (float64)：当前观测深度
 *      * depth_error   (float64)：target_depth - current_depth
 *  - Result:
 *      * success (bool) ：行为是否成功
 *      * message (string)：结果说明
 *
 * 除了 Action 通信以外，本行为还会在其私有命名空间下，可选地发布：
 *  - "/<vehicle>/<instance>/mismatch" （std_msgs/Float64），
 *       当 publish_mismatch == true 时，将 target_depth - current_depth 发布到此话题。
 *  - "/<vehicle>/<instance>/runtime"  （std_msgs/Float32），
 *       当在参数服务器中设置 publish_runtime := true 时，在执行过程中累加并发布已运行时长（秒）。
 *
 * “当前深度”由 VariableExtractor 自动订阅，保存在 variable_values_["DEPTH"] 中。
 */
class ConstantDepth
  : public behavior_manager::BehaviorWithAction<behavior_manager::DepthBehaviorAction>
{
public:
  /**
   * @brief 构造函数
   * @param instance_name 由 BehaviorManager 在加载该插件时传入的唯一实例名称
   */
  explicit ConstantDepth(const std::string& instance_name);

  virtual ~ConstantDepth() = default;

  /**
   * @brief 初始化：由 BehaviorManager 在加载插件时调用
   * @param parent_nh 来自 BehaviorManager 的全局 NodeHandle
   *
   * 在此阶段，先调用父类 initialize(parent_nh) 启动 ActionServer 和通用逻辑，
   * 然后：
   *  1. 读取“publish_runtime” 私有参数
   *  2. 在私有命名空间下创建 mismatch_pub_ 和 runtime_pub_（如果对应开关为 true）
   *  3. 创建 desired_depth_pub_（全局："/<vehicle>/desired_depth_cmd"）
   *  4. 让 VariableExtractor 订阅 “DEPTH”，以便 variable_values_["DEPTH"] 始终有最新深度值
   */
  void initialize(ros::NodeHandle& parent_nh) override;

protected:
  /**
   * @brief 当收到新的 Action Goal 时被调用
   * 此时：
   *  1. 从 action_server_->acceptNewGoal() 中读取：target_depth, publish_mismatch
   *  2. 重置 elapsed_time_、置 is_active_ = true
   *  3. 调用 activate() 使状态进入 ACTIVE
   *  4. 立刻发布一次深度误差并 send initial Feedback
   */
  void onGoal() override;

  /**
   * @brief 当 Action Client 取消 Goal 时被调用
   *  1. 置 is_active_ = false
   *  2. 使用 action_server_->setPreempted() 向客户端返回取消结果
   */
  void onPreempt() override;

  /**
   * @brief 行为的核心执行逻辑，由 BehaviorManager 在主循环调用 run() 时触发
   *
   * 每次 execute():
   *  1. 发布目标深度到 "/<vehicle_name>/desired_depth_cmd"
   *  2. 如果 publish_mismatch_ 为 true，计算并发布深度误差到私有 "mismatch" 话题
   *  3. 如果 publish_runtime_ 为 true，累加 elapsed_time_ 并发布到私有 "runtime" 话题
   *  4. 通过 action_server_->publishFeedback() 发布 current_depth 和 depth_error
   *  5. 判断是否已到达目标深度（|误差| < 0.05），若是则调用 setSucceeded() 并置 is_active_ = false
   */
  void execute() override;

  /**
   * @brief 可选：超时回调，来自 BehaviorBase 的 onTimeout()
   * 当累计运行时间超过配置的 duration_limit_ 时触发。
   * 默认处理为：将 is_active_ = false，调用 setAborted()。
   */
  void onTimeout() override;

private:
  /// 目标深度（米）
  double target_depth_;

  /// 是否发布深度误差到私有话题
  bool publish_mismatch_;

  /// 是否发布累计运行时长到私有话题
  bool publish_runtime_;

  /// 累计运行时长（秒）
  double elapsed_time_;

  /// 行为是否处于活动状态
  bool is_active_;

  // ---------------- ROS 发布/订阅器 ----------------

  /// 发布目标深度给深度控制器："/<vehicle_name>/desired_depth_cmd"
  ros::Publisher desired_depth_pub_;

  /// 私有命名空间下发布深度误差："/<vehicle>/<instance>/mismatch"
  ros::Publisher mismatch_pub_;

  /// 私有命名空间下发布运行时长："/<vehicle>/<instance>/runtime"
  ros::Publisher runtime_pub_;

  // --------- 存放通过 VariableExtractor 获取的变量值 ---------

  /**
   * @brief 存储 VariableExtractor 提取到的变量值，key 为变量名。
   * 在 initialize() 中会调用：
   *   VariableExtractor::instance().subscribe("DEPTH", nh_parent_, variable_values_, debug_);
   * 之后 variable_values_["DEPTH"] 始终保存最新深度值（Z 轴）。
   */
  std::map<std::string, double> variable_values_;
};

} // namespace solo_behaviors

#endif // SOLO_BEHAVIORS_CONSTANT_DEPTH_H
