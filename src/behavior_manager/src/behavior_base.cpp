#include "behavior_manager/BehaviorBase.h"

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

namespace behavior_manager {

BehaviorBase::BehaviorBase(const std::string& instance_name)
    : instance_name_(instance_name)
{}

BehaviorBase::~BehaviorBase() {
    // 如果创建了插件加载器，释放之
    if (extractor_loader_) {
        delete extractor_loader_;
        extractor_loader_ = nullptr;
        extractor_instance_ = nullptr;
    }
}

void BehaviorBase::initialize(ros::NodeHandle& parent_nh) {
    // --- 1. 私有命名空间 & 回调队列 & AsyncSpinner ---
    nh_parent_ = parent_nh;
    nh_ = ros::NodeHandle(parent_nh, instance_name_);
    nh_.setCallbackQueue(&cb_queue_);
    spinner_.reset(new ros::AsyncSpinner(1, &cb_queue_));

    // --- 2. 读取通用参数 ---
    nh_.param<int>("priority",         priority_,       1);
    nh_.param<double>("pwt",           pwt_,            1.0);
    nh_.param<double>("duration",      duration_limit_, 0.0);
    nh_.param<bool>("duration_idle_decay", duration_idle_decay_, false);
    nh_.param<bool>("timeout_fails",   timeout_fails_,  false);
    nh_.param<bool>("enable_debug",    debug_,          false);
    nh_.param<std::string>("vehicle_name", vehicle_name_, "");

    // “variable_extractor_plugin” 参数可以在 roslaunch 中指定，
    // 默认使用 “behavior_manager/DefaultVariableExtractor”
    std::string plugin_name = "behavior_manager/DefaultVariableExtractor";
    nh_.param<std::string>("variable_extractor_plugin", plugin_name, plugin_name);

    // --- 3. 创建并发布状态 Publisher ---
    if (!vehicle_name_.empty()) {
        state_topic_ = "/" + vehicle_name_ + "/behavior_states/" + instance_name_;
        pub_state_ = nh_parent_.advertise<std_msgs::String>(state_topic_, 1, /*latch=*/true);
    }

    // --- 4. 三类定时器的创建（均默认 autostart=false） ---
    elapsed_duration_ = 0.0;
    duration_timer_ = nh_.createTimer(
        ros::Duration(0.1),
        &BehaviorBase::onDurationTimer, this,
        /*oneshot=*/false, /*autostart=*/false
    );

    elapsed_active_ = 0.0;
    active_timer_ = nh_.createTimer(
        ros::Duration(0.1),
        &BehaviorBase::onActiveTimer, this,
        /*oneshot=*/false, /*autostart=*/false
    );

    elapsed_idle_ = 0.0;
    idle_timer_ = nh_.createTimer(
        ros::Duration(0.1),
        &BehaviorBase::onIdleTimer, this,
        /*oneshot=*/false, /*autostart=*/false
    );

    // --- 5. duration_reset 订阅 + duration_status 发布 ---
    sub_duration_reset_ = nh_.subscribe<std_msgs::Empty>(
        "duration_reset", 1,
        &BehaviorBase::onDurationReset, this
    );
    pub_duration_status_ = nh_.advertise<std_msgs::Float32>(
        "duration_status", 5, /*latch=*/true
    );

    // --- 6. 解析并订阅“通用条件”：STATE:other & VAR:op+value ---
    std::vector<std::string> cond_list;
    if (nh_.getParam("conditions", cond_list)) {
        parseConditions(cond_list);
    }
    // 6.1 订阅其他行为状态
    for (auto& sc : state_conditions_) {
        const std::string& other = sc.behavior_name;
        if (subscribed_behavior_state_topics_.count(other) == 0) {
            std::string topic = "/" + vehicle_name_ + "/behavior_states/" + other;
            auto cb = [this, other](const std_msgs::String::ConstPtr& msg) {
                BehaviorState s = stringToState(msg->data);
                other_behavior_states_[other] = s;
                if (debug_) {
                    ROS_DEBUG_STREAM("[" << instance_name_
                                     << "] other behavior [" << other
                                     << "] state → " << msg->data);
                }
            };
            ros::Subscriber sub = nh_parent_.subscribe<std_msgs::String>(
                topic, 1, cb
            );
            subscribed_behavior_state_topics_.insert(other);
            other_state_subs_.push_back(sub);
            other_behavior_states_[other] = BehaviorState::IDLE;
        }
    }

    // 6.2 通过 pluginlib 加载 VariableExtractor 插件，然后订阅变量
    try {
        extractor_loader_ = new pluginlib::ClassLoader<
            variable_extractor::VariableExtractorInterface >(
                "behavior_manager",
                "variable_extractor::VariableExtractorInterface"
        );
        extractor_instance_ = extractor_loader_->createUnmanagedInstance(plugin_name);
        extractor_instance_->setVehicleName(vehicle_name_);

        for (auto& vc : var_conditions_) {
            extractor_instance_->subscribe(vc.var_name,
                                          nh_parent_,
                                          variable_values_,
                                          debug_);
        }
    }
    catch (pluginlib::LibraryLoadException &ex) {
        ROS_ERROR("Failed to load plugin [%s]: %s", plugin_name.c_str(), ex.what());
    }
    catch (pluginlib::CreateClassException &ex) {
        ROS_ERROR("Failed to create instance of [%s]: %s", plugin_name.c_str(), ex.what());
    }

    if (debug_) {
        ROS_DEBUG_STREAM("[" << instance_name_ << "] initialize() 完成："
                         << " priority=" << priority_
                         << ", pwt=" << pwt_
                         << ", duration_limit=" << duration_limit_
                         << ", vehicle_name=" << vehicle_name_
                         << ", plugin=" << plugin_name);
    }
}

void BehaviorBase::activate() {
    if (debug_) {
        ROS_DEBUG_STREAM("[" << instance_name_ << "] activate()");
    }
    state_ = BehaviorState::ACTIVE;

    // 清零计时器
    elapsed_duration_ = 0.0;
    elapsed_active_   = 0.0;
    idle_timer_.stop();

    // 启动 ACTIVE & DURATION 计时
    active_timer_.start();
    if (duration_limit_ > 0.0) {
        duration_timer_.start();
    }
    spinner_->start();

    publishState("ACTIVE");
    onActivate();
}

void BehaviorBase::run() {
    if (state_ == BehaviorState::ACTIVE || state_ == BehaviorState::RUNNING) {
        try {
            state_ = BehaviorState::RUNNING;
            execute();

            // 如果还是 RUNNING，就检查超时
            if (state_ == BehaviorState::RUNNING &&
                duration_limit_ > 0.0 &&
                elapsed_duration_ >= duration_limit_) {
                if (debug_) {
                    ROS_WARN_STREAM("[" << instance_name_
                                     << "] 超时 (" << elapsed_duration_ << "s)");
                }
                onTimeout();
            }

            if (state_ == BehaviorState::COMPLETED) {
                publishState("COMPLETED");
                onComplete();
            }
            else if (state_ == BehaviorState::FAILED) {
                publishState("FAILED");
                onFailure();
            }
        }
        catch (const std::exception& e) {
            ROS_ERROR_STREAM("[" << instance_name_ << "] execute() 异常: " << e.what());
            state_ = BehaviorState::FAILED;
            publishState("FAILED");
            onFailure();
        }
    }
}

void BehaviorBase::deactivate() {
    if (debug_) {
        ROS_DEBUG_STREAM("[" << instance_name_ << "] deactivate()");
    }
    // 停止 ACTIVE 计时
    active_timer_.stop();
    if (duration_limit_ > 0.0 && !duration_idle_decay_) {
        duration_timer_.stop();
    }
    idle_timer_.start();

    state_ = BehaviorState::IDLE;
    publishState("IDLE");
    onDeactivate();
}

void BehaviorBase::cleanup() {
    if (debug_) {
        ROS_DEBUG_STREAM("[" << instance_name_ << "] cleanup(), 最终状态="
                         << static_cast<int>(state_));
    }
    if (duration_timer_.hasStarted()) duration_timer_.stop();
    if (active_timer_.hasStarted())   active_timer_.stop();
    if (idle_timer_.hasStarted())     idle_timer_.stop();
    spinner_->stop();

    if (state_ == BehaviorState::COMPLETED) {
        publishState("COMPLETED");
        onComplete();
    }
    else if (state_ == BehaviorState::FAILED) {
        publishState("FAILED");
        onFailure();
    }
}

bool BehaviorBase::checkCondition() const {
    if (hasOverrideCheckCondition_) {
        return this->checkConditionImpl();
    }
    if (state_conditions_.empty() && var_conditions_.empty()) {
        return true;
    }
    // 1. 检查其他行为状态条件
    for (const auto& sc : state_conditions_) {
        auto it = other_behavior_states_.find(sc.behavior_name);
        if (it == other_behavior_states_.end()
         || it->second != sc.desired_state) {
            return false;
        }
    }
    // 2. 检查变量阈值条件
    for (const auto& vc : var_conditions_) {
        auto itv = variable_values_.find(vc.var_name);
        if (itv == variable_values_.end() || std::isnan(itv->second)) {
            return false;
        }
        double val = itv->second;
        bool ok = false;
        if (vc.op == "<")  ok = (val < vc.threshold);
        if (vc.op == "<=") ok = (val <= vc.threshold);
        if (vc.op == ">")  ok = (val > vc.threshold);
        if (vc.op == ">=") ok = (val >= vc.threshold);
        if (vc.op == "==") ok = (std::fabs(val - vc.threshold) < 1e-6);
        if (!ok) return false;
    }
    return true;
}

void BehaviorBase::onActivate()   { }
void BehaviorBase::onDeactivate() { }
void BehaviorBase::onComplete()   { }
void BehaviorBase::onFailure()    { }

void BehaviorBase::onTimeout() {
    if (debug_) {
        ROS_WARN_STREAM("[" << instance_name_ << "] onTimeout() 触发");
    }
    if (timeout_fails_) {
        state_ = BehaviorState::FAILED;
        publishState("FAILED");
        onFailure();
    } else {
        state_ = BehaviorState::COMPLETED;
        publishState("COMPLETED");
        onComplete();
    }
}

void BehaviorBase::onDurationTimer(const ros::TimerEvent&) {
    if (duration_limit_ <= 0.0) return;
    bool shouldCount = (state_ == BehaviorState::RUNNING ||
                        (duration_idle_decay_ && state_ == BehaviorState::IDLE));
    if (shouldCount) {
        elapsed_duration_ += 0.1;
        if (debug_) {
            ROS_DEBUG_STREAM("[" << instance_name_ 
                             << "] duration_elapsed=" << elapsed_duration_);
        }
        if (pub_duration_status_) {
            std_msgs::Float32 msg;
            msg.data = static_cast<float>(elapsed_duration_);
            pub_duration_status_.publish(msg);
        }
    }
}

void BehaviorBase::onActiveTimer(const ros::TimerEvent&) {
    if (state_ == BehaviorState::ACTIVE || state_ == BehaviorState::RUNNING) {
        elapsed_active_ += 0.1;
        if (debug_) {
            ROS_DEBUG_STREAM("[" << instance_name_
                             << "] active_elapsed=" << elapsed_active_);
        }
    }
}

void BehaviorBase::onIdleTimer(const ros::TimerEvent&) {
    if (state_ == BehaviorState::IDLE) {
        elapsed_idle_ += 0.1;
        if (debug_) {
            ROS_DEBUG_STREAM("[" << instance_name_
                             << "] idle_elapsed=" << elapsed_idle_);
        }
    }
}

void BehaviorBase::onDurationReset(const std_msgs::Empty::ConstPtr&) {
    elapsed_duration_ = 0.0;
    if (debug_) {
        ROS_DEBUG_STREAM("[" << instance_name_ << "] duration reset = 0");
    }
}

void BehaviorBase::onOtherBehaviorState(const std_msgs::String::ConstPtr& msg,
                                        const std::string& other_name) {
    BehaviorState s = stringToState(msg->data);
    other_behavior_states_[other_name] = s;
    if (debug_) {
        ROS_DEBUG_STREAM("[" << instance_name_ 
                         << "] OtherBehavior [" << other_name 
                         << "] state = " << msg->data);
    }
}

void BehaviorBase::onVariableValue(const std_msgs::Float32::ConstPtr& msg,
                                   const std::string& var_name) {
    variable_values_[var_name] = msg->data;
    if (debug_) {
        ROS_DEBUG_STREAM("[" << instance_name_ 
                         << "] Variable [" << var_name 
                         << "] = " << msg->data);
    }
}

void BehaviorBase::setCompleted() {
    state_ = BehaviorState::COMPLETED;
    publishState("COMPLETED");
    onComplete();
}

void BehaviorBase::setFailed() {
    state_ = BehaviorState::FAILED;
    publishState("FAILED");
    onFailure();
}

void BehaviorBase::resetActiveTime() {
    elapsed_active_ = 0.0;
}

void BehaviorBase::resetIdleTime() {
    elapsed_idle_ = 0.0;
}

void BehaviorBase::startActiveTimer() {
    if (!active_timer_.hasStarted()) active_timer_.start();
}

void BehaviorBase::stopActiveTimer() {
    if (active_timer_.hasStarted()) active_timer_.stop();
}

void BehaviorBase::startIdleTimer() {
    if (!idle_timer_.hasStarted()) idle_timer_.start();
}

void BehaviorBase::stopIdleTimer() {
    if (idle_timer_.hasStarted()) idle_timer_.stop();
}

void BehaviorBase::trim(std::string& s) {
    auto is_space = [](int c){ return std::isspace(c); };
    s.erase(s.begin(), std::find_if_not(s.begin(), s.end(), is_space));
    s.erase(std::find_if_not(s.rbegin(), s.rend(), is_space).base(), s.end());
}

void BehaviorBase::parseConditions(const std::vector<std::string>& cond_list) {
    for (const auto& item : cond_list) {
        auto pos = item.find(':');
        if (pos == std::string::npos) continue;
        std::string lhs = item.substr(0, pos);
        std::string rhs = item.substr(pos + 1);
        trim(lhs);
        trim(rhs);

        std::string u = lhs;
        std::transform(u.begin(), u.end(), u.begin(), ::toupper);
        if (u == "IDLE" || u == "ACTIVE" || u == "RUNNING" ||
            u == "COMPLETED" || u == "FAILED") {
            StateCondition sc;
            sc.behavior_name = rhs;
            sc.desired_state  = stringToState(u);
            state_conditions_.push_back(sc);
        }
        else {
            VarCondition vc;
            vc.var_name = lhs;
            static const std::vector<std::string> ops = { "<=", ">=", "<", ">", "==" };
            bool found = false;
            for (auto& op : ops) {
                if (rhs.find(op) == 0) {
                    vc.op        = op;
                    vc.threshold = std::stod(rhs.substr(op.size()));
                    found = true;
                    break;
                }
            }
            if (found) {
                var_conditions_.push_back(vc);
            } else {
                ROS_WARN_STREAM("[" << instance_name_ << "] 无效的变量条件: " << item);
            }
        }
    }
}

void BehaviorBase::publishState(const std::string& s) {
    if (pub_state_) {
        std_msgs::String msg;
        msg.data = s;
        pub_state_.publish(msg);
    }
}

}  // namespace behavior_manager
