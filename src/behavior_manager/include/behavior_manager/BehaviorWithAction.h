#ifndef BEHAVIOR_MANAGER_BEHAVIOR_WITH_ACTION_H
#define BEHAVIOR_MANAGER_BEHAVIOR_WITH_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_manager/BehaviorBase.h>

namespace behavior_manager {

/**
 * @brief BehaviorWithAction：在 BehaviorBase 基础上增加 ActionLib 支持的中间类。
 * 
 * - ActionT: 对应一个具体的 ROS Action 消息类型，例如 my_pkg/ConstHeading.action 生成的 MyActionMsg
 * - 内部持有 std::unique_ptr<SimpleActionServer<ActionT>> action_server_
 * - 在 initialize() 中负责创建并启动 ActionServer
 * - 派生类可以 override onGoal()/onPreempt() 以处理客户端发来的 goal 与 cancel
 */
template <typename ActionT>
class BehaviorWithAction : public BehaviorBase {
public:
    using ActionServer = actionlib::SimpleActionServer<ActionT>;

    BehaviorWithAction(const std::string& instance_name)
        : BehaviorBase(instance_name) {}

    virtual ~BehaviorWithAction() = default;

    /**
     * @brief 初始化时，先调用基类 initialize()，再创建并启动 ActionServer
     * 
     * - action_name: 可以直接用 instance_name_ + "/action" 作为 Action 的命名空间，
     *   也可以从 ROS 参数 server 读取
     */
    void initialize(ros::NodeHandle& parent_nh) override {
        // 先调用 BehaviorBase::initialize()，完成“无 Action 部分”的初始化
        BehaviorBase::initialize(parent_nh);

        // 构造 ActionServer，路径为: "/<vehicle_name>/<instance_name>/action"
        std::string action_ns = "/" + this->vehicle_name_ + "/" + this->getName() + "/action";
        action_server_ = std::make_unique<ActionServer>(
            this->nh_,            // 使用私有 NodeHandle，namespace 已含 instance_name_
            action_ns,
            boost::bind(&BehaviorWithAction::onGoal, this),
            boost::bind(&BehaviorWithAction::onPreempt, this),
            /*auto_start=*/ false
        );
        action_server_->start();
    }

protected:
    /**
     * @brief 当客户端发来新 Goal 时会调用本方法
     *  - 一般在这里调用 action_server_->acceptNewGoal() 取到 goal
     *  - 并可向客户端发送 feedback: action_server_->publishFeedback(...)
     *  - 如果希望立即进入 RUNNING/ACTIVE，则可调用 setActive()/setRunning() 或其他逻辑
     */
    virtual void onGoal() {
        // 例如：
        // auto goal = action_server_->acceptNewGoal();
        // 处理 goal 里面的参数，然后把行为激活：
        // this->activate();
    }

    /**
     * @brief 当客户端调用 cancel() 时会调用本方法
     *  - 一般在这里调用 action_server_->setPreempted()
     *  - 并中断当前行为逻辑（例如置状态为 FAILED 或直接 deactivate()）
     */
    virtual void onPreempt() {
        action_server_->setPreempted();
        // 如果需要中断当前行为执行，可 e.g. this->setFailed(); 或 this->deactivate();
    }

    /**
     * @brief 派生类必须实现 execute()，具体做法与 BehaviorBase 一致
     */
    virtual void execute() override = 0;

    /**
     * @brief 派生类可根据需要 override checkConditionImpl()
     *  - ACTION 模式下，通常行为在接到 goal 后就直接进入 RUNNING，无需额外条件
     */
    virtual bool checkConditionImpl() const override {
        // 默认始终允许执行，子类可以根据 goal 的某些字段做判断
        return true;
    }

    std::unique_ptr<ActionServer> action_server_;

    // 如果需要给客户端发送反馈，可使用：
    //   ActionT::Feedback feedback_msg_;
    //   action_server_->publishFeedback(feedback_msg_);
    // 需要在 execute() / onGoal() 中自行调用
};

}  // namespace behavior_manager

#endif  // BEHAVIOR_MANAGER_BEHAVIOR_WITH_ACTION_H
