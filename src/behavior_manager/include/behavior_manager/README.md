# BehaviorBase 使用说明（README）

## 目录
1. [概述](#概述)  
2. [功能与设计目标](#功能与设计目标)  
3. [安装与编译](#安装与编译)  
   1. [依赖项](#依赖项)  
   2. [CMakeLists.txt 配置示例](#CMakeLists.txt-配置示例)  
   3. [编译步骤](#编译步骤)  
4. [核心类结构](#核心类结构)  
   1. [`BehaviorBase` 类简介](#BehaviorBase-类简介)  
   2. [生命周期与线程模型](#生命周期与线程模型)  
   3. [成员变量说明](#成员变量说明)  
   4. [核心成员函数与生命周期钩子](#核心成员函数与生命周期钩子)  
   5. [计时器与超时处理](#计时器与超时处理)  
   6. [条件触发与 `VariableExtractor`](#条件触发与-VariableExtractor)  
   7. [日志与调试](#日志与调试)  
5. [YAML 配置详解](#YAML-配置详解)  
   1. [模板示例](#模板示例)  
   2. [各字段说明](#各字段说明)  
   3. [条件字段示例](#条件字段示例)  
   4. [自定义参数示例](#自定义参数示例)  
6. [使用示例](#使用示例)  
   1. [纯 `BehaviorBase` 行为（不带 Action）](#纯-BehaviorBase-行为不带-Action)  
   2. [带 Action 支持的中间类（`BehaviorWithAction`）](#带-Action-支持的中间类-BehaviorWithAction)  
7. [FAQ 常见问题](#FAQ-常见问题)  
8. [联系方式](#联系方式)  

---

## 概述
`BehaviorBase` 是一个专为 ROS1 下的水下航行器（AUV/ROV）设计的「通用行为基类」，旨在将各类“单一行为”（如定深、定速、定航向、避障、抛载等）的共性逻辑抽象出来，提供：
- **统一的生命周期框架**（Active→Running→Idle→Completed/Failed）
- **权限调度相关字段**（`priority`、`pwt`）
- **超时控制**（`duration`、`timeout_fails`、`duration_idle_decay`）
- **状态发布**（`/vehicle_name/behavior_states/<instance_name>`）
- **条件触发**（基于其他行为状态与实时变量阈值）
- **模块化参数配置**（通过 YAML 文件下发并由私有 `NodeHandle` 自动读取）
- **可扩展的子类钩子**（`onActivate`、`onDeactivate`、`onComplete`、`onFailure`、`execute`、`checkConditionImpl`）
- **多线程回调隔离**（每个行为有自己的 `ros::CallbackQueue` + `AsyncSpinner`）
- **定制化调试日志**（`enable_debug` 开关）

本 README 将详细介绍 `BehaviorBase` 的设计思路、使用方式、YAML 配置及示例，并兼顾读者在后续开发自己专属行为时的常见需求。

---

## 功能与设计目标
1. **通用行为骨架**  
   - 将大多数「一次性行为」所需的共性逻辑（生命周期管理、状态发布、超时、条件触发）集中到一个基类里。  
   - 子类仅需实现：`execute()`（具体执行逻辑）或可选地覆盖 `checkConditionImpl()`（自定义触发条件），以及额外的自定义配置参数读取。

2. **可配置化**  
   - 使用 YAML 文件对“每个行为实例”在运行前下发所有参数。BehaviorBase 通过私有 `ros::NodeHandle` 自动读取（无需子类写大量 `getParam` 代码）。  
   - 支持“全局参数”（如 `vehicle_name`），支持“行为级参数”（`priority`、`pwt`、`duration` 等）和“子类自定义参数”（如 `target_depth`、`heading_angle` 等）。

3. **并发隔离**  
   - 每个行为有独立的回调队列 `ros::CallbackQueue` 和 1 线程 `ros::AsyncSpinner`，保证即使行为处于 IDLE 状态，也能够响应“超时重置”与“条件订阅回调”。

4. **条件触发**  
   - 支持两种触发方式：  
     1. **行为状态** (`STATE:<other_behavior>`): 当 `/vehicle_name/behavior_states/<other_behavior>` 发布对应状态时触发。  
     2. **变量阈值** (`<VAR>:<op><value>`): 当 `/vehicle_name/feedback/<VAR>` 发布的数值与阈值比较成立时触发。  
   - 通过 `VariableExtractor` 单例模块自动订阅并维护“变量→最新值”的映射，子类无需关心订阅细节。

5. **灵活扩展**  
   - 如果某个行为需要“自定义触发条件”，可覆写 `checkConditionImpl()`，基类会自动跳过“YAML 条件解析”逻辑。  
   - 如果某个行为在超时后要视为“失败”而非“完成”，只要在 YAML 中 `timeout_fails: true`，基类 `onTimeout()` 自动切换到 `FAILED` 分支。

---

## 安装与编译

### 依赖项
- ROS 版本：**Melodic** 或 兼容 ROS1 的发行版  
- 依赖的 ROS Package：  
  - `roscpp`  
  - `std_msgs`  
  - `nav_msgs`  
  - `pluginlib`  
  - `actionlib`  
- 系统依赖：  
  - `class_loader`（pluginlib 内部依赖）

> **注意**：如果不需要“Action 支持”，可以忽略 `actionlib` 相关，但建议保留以便后续使用 `BehaviorWithAction`。

### CMakeLists.txt 配置示例
在你的 `behavior_manager` 包的根目录下，`CMakeLists.txt` 可以参考如下结构（已示范“头文件-only”模式和未来“添加 .cpp 后自动切换为动态库”模式）：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(behavior_manager)

## 1. 使用 C++11
add_compile_options(-std=c++11)

## 2. 查找 catkin 及 actionlib、pluginlib
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  pluginlib
  actionlib
)
find_package(class_loader REQUIRED)

## 3. 导出接口和库信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES behavior_manager                # 将来如果有 .cpp 会生成 libbehavior_manager.so
  CATKIN_DEPENDS roscpp std_msgs nav_msgs pluginlib actionlib
  DEPENDS class_loader
)

## 4. 包含头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 5. 如果 src/ 下暂时没有 .cpp，则创建 INTERFACE 头文件库；
##    如果有 .cpp，则编译成动态库
file(GLOB BEHAVIOR_CPP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

if (BEHAVIOR_CPP_SOURCES)
  message(STATUS "Found sources, building behavior_manager as a shared library.")
  add_library(behavior_manager
    ${BEHAVIOR_CPP_SOURCES}
  )
  target_link_libraries(behavior_manager
    ${catkin_LIBRARIES}
  )
else()
  message(STATUS "No .cpp found, building behavior_manager as INTERFACE (header-only).")
  add_library(behavior_manager INTERFACE)
  target_include_directories(behavior_manager INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

## 6. 安装规则
install(TARGETS behavior_manager
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  PUBLIC_HEADER DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
install(FILES behavior_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

#### 说明
- **INTERFACE 头文件库**：当 `src/*.cpp` 暂时为空时，CMake 会构造一个仅暴露头文件的 `behavior_manager` INTERFACE 库。这样可以先只写头文件、先编译通过，之后补充 `.cpp` 人才生成真正的 `.so`。  
- `behavior_plugin.xml` 放在包根目录下，用于 pluginlib 插件注册。  

### 编译步骤
1. **创建工作区**  
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone <your_repo_url> behavior_manager
   ```
2. **编译**  
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
   - 如果一切配置正常，会先编译出 `devel/lib/libbehavior_manager.so`（如果有 `.cpp`）或仅安装头文件（如果目前只写了头文件）。  
   - 同时会生成 `devel/share/behavior_manager/behavior_plugin.xml`，供 Pluginlib 在运行时加载具体行为插件。

---

## 核心类结构

### `BehaviorBase` 类简介
```cpp
namespace behavior_manager {

enum class BehaviorState {
    IDLE,       // 未激活
    ACTIVE,     // 已激活（等待/准备运行）
    RUNNING,    // 正在执行 execute()
    COMPLETED,  // 正常完成
    FAILED      // 执行失败
};

class BehaviorBase {
public:
    // 构造时传入实例名称，该名称是在 YAML 中的 "name" 字段
    BehaviorBase(const std::string& instance_name);
    virtual ~BehaviorBase() = default;

    // （1）初始化阶段
    virtual void initialize(ros::NodeHandle& parent_nh);

    // （2）激活阶段：外部由管理器调用
    virtual void activate();

    // （3）运行阶段：在主循环里被周期性调用
    void run();

    // （4）停用阶段（切回 IDLE）：外部由管理器调用
    virtual void deactivate();

    // （5）清理 / 完成：外部由管理器调用
    virtual void cleanup();

    // 获取当前状态、优先级、权重
    BehaviorState getState() const;
    int getPriority() const;
    double getPWT() const;

    // 子类必须实现：返回与 YAML 中 name 一致的实例名称
    virtual std::string getName() const;

    // 子类必须实现：核心执行函数
    virtual void execute() = 0;

    // 子类可选覆盖：自定义条件判断（基类会自动触发 parseConditions）
    virtual bool checkCondition() const;

protected:
    // ========== 生命周期钩子 ==========
    virtual void onActivate();    // 从 IDLE → ACTIVE 时调用
    virtual void onDeactivate();  // 从 ACTIVE → IDLE 时调用
    virtual void onComplete();    // 状态转为 COMPLETED 时调用
    virtual void onFailure();     // 状态转为 FAILED 时调用
    virtual void onTimeout();     // 超时触发：默认把状态设为 COMPLETED，但可根据 timeout_fails_ 设为 FAILED

    // ========== 计时器回调 ==========
    virtual void onDurationTimer(const ros::TimerEvent&);
    virtual void onActiveTimer(const ros::TimerEvent&);
    virtual void onIdleTimer(const ros::TimerEvent&);
    virtual void onDurationReset(const std_msgs::Empty::ConstPtr&);

    // ========== 条件回调 ==========
    virtual void onOtherBehaviorState(const std_msgs::String::ConstPtr&, const std::string& other_name);
    virtual void onVariableValue(const std_msgs::Float32::ConstPtr&, const std::string& var_name);

    // ========== 辅助方法 ==========
    void setCompleted();
    void setFailed();
    void resetActiveTime();
    void resetIdleTime();
    void publishState(const std::string& s);

    // 子类可选覆盖：自定义条件逻辑
    virtual bool checkConditionImpl() const;

private:
    // 解析 YAML 中的 conditions 字符串到 state_conditions_ 与 var_conditions_
    void parseConditions(const std::vector<std::string>& cond_list);
    static void trim(std::string& s);

    // 私有成员变量列表见下面“成员变量说明”
};

}  // namespace behavior_manager
```

#### 设计要点
- **统一生命周期**：由外部管理器（BehaviorManager）在合适时机依次调用 `activate()` → `run()`（循环） → `deactivate()` → `cleanup()`。  
- **线程隔离**：在 `initialize()` 中为每个实例创建私有 `ros::CallbackQueue cb_queue_` 和 `ros::AsyncSpinner spinner_`，确保即使该行为处于 IDLE，也能独立接收“超时复位”与“条件订阅回调”。  
- **状态发布**：在 `activate()`、`onComplete()`、`onFailure()`、`deactivate()` 等关键点调用 `publishState("ACTIVE"/"COMPLETED"/"FAILED"/"IDLE")`，发送 `std_msgs::String` 到 `/vehicle_name/behavior_states/<instance_name>`。

---

### 生命周期与线程模型

1. **`initialize(ros::NodeHandle& parent_nh)`**  
   - 创建私有 `NodeHandle nh_(parent_nh, instance_name_)` 并指定独立 `CallbackQueue`。  
   - 读取所有通用参数（`priority`, `pwt`, `duration`, `duration_idle_decay`, `timeout_fails`, `enable_debug`, `vehicle_name`）。  
   - 创建三类定时器：  
     - `duration_timer_`（10Hz）负责累计 `elapsed_duration_`。  
     - `active_timer_`（10Hz）负责累计 `elapsed_active_`。  
     - `idle_timer_`（10Hz）负责累计 `elapsed_idle_`。  
   - 订阅“持续时间复位”话题 `/instance_name/duration_reset`。  
   - 发布“持续时间状态”话题 `/instance_name/duration_status`。  
   - 解析并订阅 `conditions:` 对应的“状态条件”与“变量条件”。  
   - 发布状态话题 `/vehicle_name/behavior_states/<instance_name>` 的 `ros::Publisher pub_state_`。  

2. **`activate()`**  
   - 将状态设为 `ACTIVE`。  
   - 清零累计：`elapsed_duration_`, `elapsed_active_`。  
   - 停止 `idle_timer_`，启动 `active_timer_`（和 `duration_timer_`，如果 `duration > 0`）。  
   - 启动 `spinner_->start()`。  
   - 发布 `"ACTIVE"` 到 `/vehicle_name/behavior_states/<instance_name>`。  
   - 调用子类 `onActivate()`。  

3. **`run()`**  
   - 在 `state_ == ACTIVE || state_ == RUNNING` 时：  
     1. `state_ = RUNNING`，调用 `execute()`。  
     2. 若仍 `RUNNING` 且 `elapsed_duration_ >= duration_limit_`，调用 `onTimeout()`.  
     3. 若 `state_ == COMPLETED`，发布 `"COMPLETED"` 并调用 `onComplete()`；  
        若 `state_ == FAILED`，发布 `"FAILED"` 并调用 `onFailure()`。  

4. **`deactivate()`**  
   - 停止 `active_timer_`，（若 `duration_limit_ > 0 && !duration_idle_decay_`）停止 `duration_timer_`。  
   - `state_ = IDLE`，发布 `"IDLE"`，调用 `onDeactivate()`。  
   - 启动 `idle_timer_`。  

5. **`cleanup()`**  
   - 停止所有定时器与 `spinner_`。  
   - 如果 `state_ == COMPLETED`，发布 `"COMPLETED"`, 调用 `onComplete()`。  
     如果 `state_ == FAILED`，发布 `"FAILED"`, 调用 `onFailure()`。  

---

### 成员变量说明

| 成员名                                | 类型                                                    | 说明                                                                                                                                                                 |
|---------------------------------------|---------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `ros::NodeHandle nh_parent_;`         | ROS NodeHandle                                          | 父节点句柄，通常是由 BehaviorManager 传入。                                                                                                                           |
| `ros::NodeHandle nh_;`                | ROS NodeHandle                                          | 私有命名空间句柄，namespace = `<instance_name>`。                                                                                                                     |
| `ros::CallbackQueue cb_queue_;`       | ROS CallbackQueue                                       | 私有回调队列，用于隔离该行为的订阅回调与定时器回调。                                                                                                                   |
| `std::unique_ptr<ros::AsyncSpinner> spinner_;` | 异步 Spinner                                            | 对应私有回调队列的异步线程，用于在行为 ACTIVE 或 RUNNING 时处理回调。                                                                                                  |
| `std::string instance_name_;`         | 字符串                                                  | 行为实例名称，与 YAML 中的 `name` 字段一致；用于生成私有命名空间及状态话题名称。                                                                                       |
| `std::string vehicle_name_;`          | 字符串                                                  | 车辆名称顶层参数，BehaviorBase 用它拼 `/vehicle_name/behavior_states/<instance_name>` 及 `/vehicle_name/feedback/<VAR>` 等全局话题。                                       |
| `int priority_;`                      | 整数                                                    | 优先级等级，值越小代表优先级越高（0 为最高）。管理器按此做抢占逻辑。                                                                                                   |
| `double pwt_;`                        | 浮点                                                    | 同一优先级下的权重，值越大代表倾向度更高；管理器可据此判断多个同级行为的相对优先顺序。                                                                                   |
| `double duration_limit_;`             | 浮点                                                    | 最大持续时长（秒），当累计运行时长 ≥ 该值时触发 `onTimeout()`；≤0 表示不检查。                                                                                          |
| `bool duration_idle_decay_;`           | 布尔                                                    | 当行为处于 IDLE 时，是否继续累积 `elapsed_duration_`；常见场景一般为 false。                                                                                           |
| `bool timeout_fails_;`                 | 布尔                                                    | 超时后是否算作失败；若 true 则 `onTimeout()` 会把状态设为 `FAILED` 并调用 `onFailure()`；否则设为 `COMPLETED` 并调用 `onComplete()`.                                      |
| `bool debug_;`                         | 布尔                                                    | 是否开启调试日志（`ROS_DEBUG`）。若 true，将在计时、条件触发、状态更新等多个环节输出详细信息。                                                                            |
| `ros::Timer duration_timer_;`          | ROS 定时器                                              | 10Hz 触发 `onDurationTimer()`，用于累计 `elapsed_duration_`.                                                                                                            |
| `double elapsed_duration_;`            | 浮点                                                    | 累计的“运行时长”（秒）。在 ACTIVE/RUNNING 或（IDLE + `duration_idle_decay_ == true`）时递增。                                                                             |
| `ros::Timer active_timer_;`            | ROS 定时器                                              | 10Hz 触发 `onActiveTimer()`，用于累计 `elapsed_active_`，即“行为 ACTIVE 状态下消耗的时间”。                                                                               |
| `double elapsed_active_;`              | 浮点                                                    | 累计行为处于 ACTIVE（或 RUNNING）状态消耗的秒数。                                                                                                                       |
| `ros::Timer idle_timer_;`              | ROS 定时器                                              | 10Hz 触发 `onIdleTimer()`，用于累计 `elapsed_idle_`，即“行为 IDLE 状态下消耗的时间”。                                                                                     |
| `double elapsed_idle_;`                | 浮点                                                    | 累计行为处于 IDLE 状态消耗的秒数。                                                                                                                                       |
| `ros::Subscriber sub_duration_reset_;` | ROS 订阅器                                              | 订阅私有话题 `/ <instance_name> / duration_reset`（`std_msgs/Empty`），一旦收到则调用 `onDurationReset()` 归零计时。                                                      |
| `ros::Publisher pub_duration_status_;` | ROS 发布器                                              | 发布私有话题 `/ <instance_name> / duration_status` (`std_msgs/Float32`)，用于外部“状态监视”或日志。                                                                        |
| `std::vector<StateCondition> state_conditions_;` | 自定义结构体列表                                        | 存放 YAML 中解析后的“状态触发条件”列表，每项包含 `behavior_name` 与 `desired_state`（枚举）。                                                                             |
| `std::vector<VarCondition> var_conditions_;`     | 自定义结构体列表                                        | 存放 YAML 中解析后的“变量触发条件”列表，每项包含 `var_name`、`op`、`threshold`。                                                                                          |
| `std::map<std::string, BehaviorState> other_behavior_states_;` | 映射表                                                    | 存储所有“被订阅行为”最新状态，以便在 `checkCondition()` 中快速判断是否触发。                                                                                               |
| `std::map<std::string, double> variable_values_;`           | 映射表                                                    | 存储所有“被订阅变量”最新值，由 `VariableExtractor` 更新，用于条件判断。                                                                                                   |
| `std::set<std::string> subscribed_behavior_state_topics_;` | 字符串集合                                                | 防止重复订阅“其他行为状态”话题时，多次创建 Subscriber。                                                                                                                      |
| `std::set<std::string> subscribed_variable_topics_;`       | 字符串集合                                                | 防止重复订阅“反馈变量”话题时，多次创建 Subscriber。                                                                                                                        |
| `std::vector<ros::Subscriber> other_state_subs_;`          | Subscriber 列表                                           | 用于保存“其他行为状态”订阅器，以防它们被析构。                                                                                                                             |
| `std::vector<ros::Subscriber> variable_subs_;`             | Subscriber 列表                                           | 用于保存“变量订阅”订阅器，以防它们被析构。                                                                                                                             |
| `std::string state_topic_;`                | 字符串                                                  | 最终的“状态发布”话题名称，形如 `"/<vehicle_name>/behavior_states/<instance_name>"`。                                                                                         |
| `ros::Publisher pub_state_;`                | 发布器                                                  | 用于发布 `std_msgs::String` 类型的当前行为状态（`"IDLE"`/`"ACTIVE"`/`"RUNNING"`/`"COMPLETED"`/`"FAILED"`）。                                                                   |
| `BehaviorState state_;`                     | 枚举                                                    | 当前行为状态，初始值为 `BehaviorState::IDLE`。                                                                                                                              |
| `bool hasOverrideCheckCondition_;`          | 布尔                                                    | 标记子类是否覆盖了 `checkConditionImpl()`，如果覆盖则直接调用子类逻辑跳过基类“通用条件”判断。                                                                                  |

---

### 核心成员函数与生命周期钩子

#### `initialize(ros::NodeHandle& parent_nh)`
- **功能**：  
  1. 创建私有 `NodeHandle nh_(parent_nh, instance_name_)` 并指定独立 `CallbackQueue`。  
  2. 启动 1 线程 `AsyncSpinner`。  
  3. 从 `nh_` 下读取通用参数：  
     - `priority`（整数）  
     - `pwt`（浮点）  
     - `duration`（浮点）  
     - `duration_idle_decay`（布尔）  
     - `timeout_fails`（布尔）  
     - `enable_debug`（布尔）  
     - `vehicle_name`（字符串）  
  4. 根据 `vehicle_name` 拼接私有状态发布话题 `/vehicle_name/behavior_states/instance_name` 并创建 `ros::Publisher pub_state_`。  
  5. 创建并不自动启动的三类 `ros::Timer`：  
     - `duration_timer_`（10Hz，触发 `onDurationTimer`）  
     - `active_timer_`（10Hz，触发 `onActiveTimer`）  
     - `idle_timer_`（10Hz，触发 `onIdleTimer`）  
  6. 订阅私有话题 `/instance_name/duration_reset`，对应回调 `onDurationReset`。  
  7. 创建私有发布 `/instance_name/duration_status`，用于外部“状态监视”或日志。  
  8. 解析 `nh_.getParam("conditions", cond_list)` 得到的 `vector<string>`，调用 `parseConditions`（拆分成状态与变量两类条件）。  
  9. 对每个状态型条件 `{ behavior_name, desired_state }`，在 `/vehicle_name/behavior_states/behavior_name` 上动态创建订阅，并初始化 `other_behavior_states_[behavior_name] = IDLE`。  
  10. 对每个变量型条件 `{ var_name, op, threshold }`，调用 `VariableExtractor::instance().subscribe(var_name, nh_parent_, variable_values_, debug_)`，由它自动在 `/vehicle_name/feedback/var_name` 上创建订阅器，并在 `variable_values_[var_name]` 中存储最新数值。  

#### `activate()`
- **功能**：  
  1. 如果 `debug_ == true`，打印 `"[instance_name] activate()"` 的 ROS_DEBUG。  
  2. `state_ = BehaviorState::ACTIVE`。  
  3. 清零所有计时器累计：`elapsed_duration_ = 0.0`，`elapsed_active_ = 0.0`；停止 `idle_timer_`。  
  4. 启动 `active_timer_`，如果 `duration_limit_ > 0`，则也启动 `duration_timer_`。  
  5. 启动 `spinner_->start()`，开始处理私有回调队列。  
  6. 发布一次 `"ACTIVE"` 到 `/vehicle_name/behavior_states/<instance_name>`。  
  7. 调用子类钩子 `onActivate()`。  

#### `run()`
- **功能**：  
  1. 只有当 `state_ == ACTIVE || state_ == RUNNING` 时才执行内部逻辑。  
  2. `state_ = BehaviorState::RUNNING` 并调用子类 `execute()`。  
  3. 如果 `state_` 仍为 `RUNNING`，且 `duration_limit_ > 0`，检查 `elapsed_duration_ >= duration_limit_`：如果超时，调用 `onTimeout()`。  
  4. 如果子类 `execute()` 或 `onTimeout()` 将 `state_` 设为 `COMPLETED`，调用 `publishState("COMPLETED")` + `onComplete()`；  
     如果将 `state_` 设为 `FAILED`，则调用 `publishState("FAILED")` + `onFailure()`；  
     如果发生异常，捕获后也把状态设为 `FAILED`，并调用 `onFailure()`。  

#### `deactivate()`
- **功能**：  
  1. 如果 `debug_ == true`，打印 `"[instance_name] deactivate()"`。  
  2. 停止 `active_timer_`，如果 `duration_limit_ > 0 && !duration_idle_decay_`，则停止 `duration_timer_`。  
  3. `state_ = BehaviorState::IDLE`，调用 `publishState("IDLE")`，并触发子类 `onDeactivate()`。  
  4. 启动 `idle_timer_`。  

#### `cleanup()`
- **功能**：  
  1. 如果 `debug_ == true`，打印 `"[instance_name] cleanup(), finalState=X"`。  
  2. 停止所有计时器：`duration_timer_`、`active_timer_`、`idle_timer_`。停止 `spinner_`。  
  3. 如果 `state_ == COMPLETED`，调用 `publishState("COMPLETED")` + `onComplete()`；  
     如果 `state_ == FAILED`，调用 `publishState("FAILED")` + `onFailure()`。  

#### `checkCondition()`
- **功能**：  
  1. 如果子类覆盖了 `checkConditionImpl()`（由 `hasOverrideCheckCondition_` 标记），则直接调用子类实现。  
  2. 否则，基类判断：  
     - 如果 `state_conditions_` 与 `var_conditions_` 均为空，则直接返回 `true`（无须等待条件）。  
     - 对于每个 `state_conditions_` 项：  
       ```cpp
       if (other_behavior_states_[sc.behavior_name] != sc.desired_state) return false;
       ```  
     - 对于每个 `var_conditions_` 项：  
       ```cpp
       auto itv = variable_values_.find(vc.var_name);
       if (itv == variable_values_.end() || std::isnan(itv->second)) {
           return false;
       }
       double val = itv->second;
       bool ok = false;
       if      (vc.op == "<")  ok = (val < vc.threshold);
       else if (vc.op == "<=") ok = (val <= vc.threshold);
       else if (vc.op == ">")  ok = (val > vc.threshold);
       else if (vc.op == ">=") ok = (val >= vc.threshold);
       else if (vc.op == "==") ok = (std::fabs(val - vc.threshold) < 1e-6);
       if (!ok) return false;
       ```
       只有当所有 `state_conditions_` 和 `var_conditions_` 都满足时，才返回 `true`。  

#### 子类可选覆盖的方法
- `virtual bool checkConditionImpl() const`：如果子类需要**完全自定义触发条件**，可 override 本函数并返回自定义逻辑。  
- `virtual void onActivate()`  
- `virtual void onDeactivate()`  
- `virtual void onComplete()`  
- `virtual void onFailure()`  
- `virtual void onTimeout()`：默认 “超时 → COMPLETED → onComplete()”，如果 `timeout_fails_ == true`，则 “超时 → FAILED → onFailure()”。如果业务上想改成“超时先停机做其它逻辑”，可由子类 override。  

---

### 计时器与超时处理

BehaviorBase 中维护了三组计时器与累计变量，分别用于统计行为处于不同状态时消耗的时长，典型用途如下：

1. **`duration_timer_` + `elapsed_duration_`**  
   - 10 Hz 触发 `onDurationTimer()`。  
   - 如果 `duration_limit_ > 0`，且行为处于 `RUNNING` 状态，或者 `duration_idle_decay_ == true` 且处于 `IDLE`，则累计 `elapsed_duration_ += 0.1`。  
   - 每次 `elapsed_duration_` 更新后，如果有 `pub_duration_status_`，则发布一个 `std_msgs::Float32` 通知当前累计时长。  
   - 在 `run()` 中，如果 `state_ == RUNNING && elapsed_duration_ >= duration_limit_`，则触发 `onTimeout()`。

2. **`active_timer_` + `elapsed_active_`**  
   - 10 Hz 触发 `onActiveTimer()`。  
   - 仅当 `state_ == ACTIVE || state_ == RUNNING` 时，才累计 `elapsed_active_ += 0.1`。  
   - 用于记录行为自从激活（ACTIVE）开始，到停用（IDLE）前一共花了多少时间。行为管理器或调试时可查询。

3. **`idle_timer_` + `elapsed_idle_`**  
   - 10 Hz 触发 `onIdleTimer()`。  
   - 仅当 `state_ == IDLE` 时，才累计 `elapsed_idle_ += 0.1`。  
   - 用于记录行为处于“空闲”状态多久，可能在调度算法中判断“行为空闲过久后做哪些操作”等。

4. **`onTimeout()` 默认逻辑**  
   ```cpp
   virtual void onTimeout() {
       if (debug_) ROS_WARN_STREAM("[" << instance_name_ << "] onTimeout() 触发");
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
   ```
   - 通过在 YAML 里设置 `timeout_fails: true/false`，操作人员可以灵活决定“超时后是算完成”还是“超时算失败”。

---

### 条件触发与 `VariableExtractor`

#### “行为状态”条件 (`STATE:<other_behavior>`)
- YAML 写法举例：  
  ```yaml
  conditions:
    - "RUNNING:waypoint_1"  
    - "COMPLETED:GOtoDepth_2"
  ```
- 在 `parseConditions()` 阶段，`lhs = "RUNNING"`（状态名），`rhs = "waypoint_1"`（另一个行为实例名）。基类把它加入 `state_conditions_`：  
  ```cpp
  StateCondition sc;
  sc.behavior_name  = "waypoint_1";
  sc.desired_state  = BehaviorState::RUNNING;
  state_conditions_.push_back(sc);
  ```
- 在 `initialize()` 中，针对每个 `state_conditions_` 中的 `behavior_name`，基类会动态创建订阅：  
  ```cpp
  std::string topic = "/" + vehicle_name_ + "/behavior_states/" + behavior_name;
  ros::Subscriber sub = nh_parent_.subscribe<std_msgs::String>(
      topic, 1,
      [this, behavior_name](const std_msgs::String::ConstPtr& msg) {
          BehaviorState s = stringToState(msg->data);
          other_behavior_states_[behavior_name] = s;
      });
  other_behavior_states_[behavior_name] = BehaviorState::IDLE; // 默认
  ```
- 在 `checkCondition()` 中，遍历 `state_conditions_`：  
  ```cpp
  auto it = other_behavior_states_.find(sc.behavior_name);
  if (it == other_behavior_states_.end() || it->second != sc.desired_state)
      return false;
  ```
  只有当被订阅行为发布 **最新** 状态等于 `sc.desired_state` 时，条件才算满足。

#### “变量阈值”条件 (`<VAR>:<op><value>`)
- YAML 写法举例：  
  ```yaml
  conditions:
    - "DEPTH:<3.0"
    - "SPEED:>=0.5"
  ```
- 在 `parseConditions()` 阶段，`lhs = "DEPTH"`，`rhs = "<3.0"`。基类把它加入 `var_conditions_`：  
  ```cpp
  VarCondition vc;
  vc.var_name  = "DEPTH";
  vc.op        = "<";
  vc.threshold = 3.0;
  var_conditions_.push_back(vc);
  ```
- 在 `initialize()` 中，基类调用：  
  ```cpp
  VariableExtractor::instance().setVehicleName(vehicle_name_);
  for (auto& vc : var_conditions_) {
      VariableExtractor::instance().subscribe(
          vc.var_name, nh_parent_, variable_values_, debug_);
  }
  ```
  - `VariableExtractor` 单例会在 `/vehicle_name/feedback/<var_name>` 上创建一个 `ros::Subscriber`，并在回调里更新 `variable_values_[var_name] = msg->data`。  
  - 默认初始值 `variable_values_[var_name] = 0.0`；如果订阅不到或尚未收到数据，则会一直是 `0.0` 或 NaN，取决于实际实现。  

- 在 `checkCondition()` 中，遍历 `var_conditions_`：  
  ```cpp
  auto itv = variable_values_.find(vc.var_name);
  if (itv == variable_values_.end() || std::isnan(itv->second)) {
      return false;
  }
  double val = itv->second;
  bool ok = false;
  if      (vc.op == "<")  ok = (val < vc.threshold);
  else if (vc.op == "<=") ok = (val <= vc.threshold);
  else if (vc.op == ">")  ok = (val > vc.threshold);
  else if (vc.op == ">=") ok = (val >= vc.threshold);
  else if (vc.op == "==") ok = (std::fabs(val - vc.threshold) < 1e-6);
  if (!ok) return false;
  ```
  只有当所有 `state_conditions_` 和 `var_conditions_` 都满足时，才返回 `true`，使 `run()` 能够继续进入 `execute()`。

---

### 日志与调试
- **`enable_debug`**：  
  - 如果设置 `enable_debug: true`，则在各个关键回调（计时器、状态更新、变量更新）都会输出 `ROS_DEBUG_STREAM`。  
  - 包括：`activate()`、`deactivate()`、`onDurationTimer()`、`onActiveTimer()`、`onIdleTimer()`、`onDurationReset()`、`onOtherBehaviorState()`、`onVariableValue()` 等都会有额外日志。  
  - 有助于在行为并发、条件判断、超时等复杂场景下进行排查。  

- **`ROS_WARN` / `ROS_ERROR`**：  
  - 在 `onTimeout()`、`checkCondition()` 失败、`execute()` 异常等关键点会输出不同级别日志。  
  - 如果不想看到调试日志，只需在 YAML 中 `enable_debug: false` 或不写该字段。

---

## YAML 配置详解

下面给出“方案二”下可直接复制粘贴、填值即可使用的 YAML 模板，并针对每个字段做详细说明。

### 模板示例（可直接填写，下述注释可删除）
```yaml
################################################################################
# 【全局配置】：请务必填写 vehicle_name，用于所有行为拼全局话题前缀
################################################################################
vehicle_name: <YOUR_VEHICLE_NAME>
# 例如：vehicle_name: auv1

################################################################################
# 【行为列表】：将所有行为实例放入 behaviors 数组
################################################################################
behaviors:
  - type: <BEHAVIOR_TYPE>       # 【必填】插件类型名称（pluginlib 注册时的 C++ 类名）
                                # 示例：behavior_manager::ConstantDepth 或 ConstantDepth
    name: <INSTANCE_NAME>       # 【必填】实例名称，必须与代码中 BehaviorBase("<INSTANCE_NAME>") 保持一致
                                # 示例：constant_depth_1

    # ----------------------- 通用参数（可选） -----------------------
    # priority: <INTEGER>         # 优先级等级，值越小越优先；默认为 1
    #                              # 示例：priority: 1

    # pwt: <FLOAT>               # 同一优先级内的权重，值越大偏好度越高；默认为 1.0
    #                              # 示例：pwt: 1.0

    # duration: <FLOAT>           # 最大运行时长 (秒)， <=0 表示不检查；默认为 0.0
    #                              # 示例：duration: 10.0

    # duration_idle_decay: <true/false>  # 行为处于 IDLE 时是否继续累积 duration；默认为 false
    #                              # 示例：duration_idle_decay: false

    # timeout_fails: <true/false>   # 超时后是否视为失败；true→FAILED，否则→COMPLETED；默认为 false
    #                              # 示例：timeout_fails: true

    # enable_debug: <true/false>   # 是否开启调试日志 (ROS_DEBUG)；默认为 false
    #                              # 示例：enable_debug: true

    # ----------------------- 启动条件（可选） -----------------------
    # conditions:                  # 字符串列表，所有条件需同时满足才能激活
    #   - "<STATE>:<OTHER_INSTANCE>"   # 示例："RUNNING:waypoint_1"
    #   - "<VAR>:<op><value>"          # 示例："DEPTH:<3.0"
    #                                   "SPEED:>=0.5"
    # 如果不需要额外条件，可不写 conditions 或者写空列表：
    #   conditions: []

    # ----------------------- 自定义参数（子类专用，可选） -----------------------
    # 子类可在 initialize() 或 execute() 中使用 nh_.param<类型>("参数名", 变量, 默认值) 读取：
    #   nh_.param<double>("target_depth", target_depth_, 0.0);
    #   nh_.param<double>("heading_angle", heading_angle_, 0.0);
    #   nh_.param<double>("safe_distance", safe_distance_, 0.0);

    # target_depth: <FLOAT>       # 目标深度（米），示例：target_depth: 3.5
    # heading_angle: <FLOAT>      # 目标航向（度），示例：heading_angle: 90.0
    # safe_distance: <FLOAT>      # 避障安全距离（米），示例：safe_distance: 1.0
    # circle_radius: <FLOAT>      # 圆周运动半径（米），示例：circle_radius: 5.0
    # max_speed: <FLOAT>          # 最大速度 (m/s)，示例：max_speed: 0.2
```

### 各字段说明

#### 顶层
- **`vehicle_name`**  
  - **说明**：AUV/ROV 的名称，用于拼接所有全局话题。例如，后续会向 `/vehicle_name/behavior_states/<instance_name>` 发布状态；也会订阅 `/vehicle_name/feedback/<VAR>` 以获取实时变量。  
  - **示例**：`vehicle_name: auv1`

#### `behaviors` 数组
- **类型**：YAML 列表  
- **作用**：将所有行为实例配置放到同一个命名空间下，BehaviorManager 只需读取 `/behaviors` 数组。  
- **示例**：  
  ```yaml
  behaviors:
    - type: ConstantDepth
      name: constant_depth_1
      priority: 1
      …
    - type: GoToDepth
      name: goto_depth_1
      …
  ```

#### 每个行为元素
| 字段                  | 类型      | 说明                                                                                                    | 示例                                                       |
|-----------------------|----------|---------------------------------------------------------------------------------------------------------|------------------------------------------------------------|
| `type`                | 字符串    | **必填**。插件类型名称，对应 pluginlib 注册的 C++ 类。例如 `behavior_manager::ConstantDepth` 或 `ConstantDepth`。 | `type: behavior_manager::ConstantDepth`                      |
| `name`                | 字符串    | **必填**。实例名称，与 C++ 代码里 `BehaviorBase("<INSTANCE_NAME>")` 保持一致。拼 `/vehicle_name/behavior_states/<INSTANCE_NAME>`。 | `name: constant_depth_1`                                   |
| `priority`            | 整数      | 优先级等级，值越小越优先；默认为 1。管理器依据此做抢占逻辑。                                                   | `priority: 1`                                              |
| `pwt`                 | 浮点      | 同一优先级内的权重，值越大偏好度越高；默认为 1.0。                                                         | `pwt: 1.0`                                                 |
| `duration`            | 浮点      | 最大运行时长（秒）， <=0 表示不检查；默认为 0.0。                                                         | `duration: 10.0`                                           |
| `duration_idle_decay` | 布尔      | 行为处于 IDLE 时是否继续累积 `duration`；默认为 false。                                                    | `duration_idle_decay: false`                                |
| `timeout_fails`       | 布尔      | 超时后是否视为“失败”；true→`FAILED`，false→`COMPLETED`；默认为 false。                                       | `timeout_fails: true`                                        |
| `enable_debug`        | 布尔      | 是否开启调试日志（`ROS_DEBUG`）；默认为 false。                                                           | `enable_debug: true`                                        |
| `conditions`          | 列表      | **可选**。字符串列表，所有条件需同时满足才激活。由基类解析成“状态条件”与“变量条件”。                                 | 见下方“条件字段示例”                                          |
| **自定义参数**         | 各类型    | **子类专属**。例如 `target_depth`（浮点）、`heading_angle`（浮点）、`safe_distance`（浮点）等。子类可在 `initialize()` 中通过 `nh_.param<>()` 读取。 | `target_depth: 3.5`<br/>`heading_angle: 90.0`                 |

---

### 条件字段示例
```yaml
conditions:
  - "COMPLETED:waypoint_1" # 等待 /<vehicle_name>/behavior_states/waypoint_1 发布 COMPLETED
  - "DEPTH:<3.0"           # 等待 /<vehicle_name>/feedback/DEPTH 发布值 < 3.0
  - "SPEED:>=0.5"          # 等待 /<vehicle_name>/feedback/SPEED 发布值 ≥ 0.5
```
- **状态条件 (`STATE:<other_instance>`)：**  
  - 仅当 `<other_instance>` 行为发布相应状态（IDLE/ACTIVE/RUNNING/COMPLETED/FAILED）才满足。  
- **变量条件 (`<VAR>:<op><value>`)：**  
  - `<VAR>` 必须是 `DEPTH`、`SPEED`、`HEADING`、`PITCH`、`ROLL`。  
  - `<op>` 支持 `"<", "<=", ">", ">=", "=="`。  
  - `<value>` 为数字（浮点或整数）。  
  - 例如：`"DEPTH:<3.0"` 表示等待 `/vehicle_name/feedback/DEPTH` 的值持续小于 3.0。

---

### 自定义参数示例
以下参数仅作示例，实际可根据子类需求灵活定义、任意添加：
```yaml
target_depth: 3.5      # 目标深度（米）
heading_angle: 90.0    # 目标航向（度）
safe_distance: 1.0     # 避障安全距离（米）
circle_radius: 5.0     # 圆周运动半径（米）
max_speed: 0.2         # 最大速度 (m/s)
```
- 子类在 `initialize()` 或 `execute()` 里，通过 `nh_.param<double>("target_depth", target_depth_, default)` 读取即可。  
- 如果未在 YAML 中出现该键，则会使用指定的默认值（如例中为 `0.0`）。

---

## 使用示例

### 纯 `BehaviorBase` 行为（不带 Action）示例

#### 1. 头文件：`ConstantDepth.h`
```cpp
#ifndef BEHAVIOR_MANAGER_CONSTANT_DEPTH_H
#define BEHAVIOR_MANAGER_CONSTANT_DEPTH_H

#include "behavior_manager/BehaviorBase.h"

namespace behavior_manager {

/**
 * @brief ConstantDepth：保持恒定深度的行为示例（不带 Action）
 */
class ConstantDepth : public BehaviorBase {
public:
    ConstantDepth(const std::string& name)
        : BehaviorBase(name) {}

    virtual ~ConstantDepth() = default;

    // 初始化时读取私有参数
    virtual void initialize(ros::NodeHandle& parent_nh) override {
        BehaviorBase::initialize(parent_nh);  
        nh_.param<double>("target_depth", target_depth_, 0.0);
        nh_.param<double>("completion_thresh", completion_thresh_, 0.1);
        if (debug_) {
            ROS_DEBUG_STREAM("[" << instance_name_ 
                               << "] target_depth=" << target_depth_
                               << ", completion_thresh=" << completion_thresh_);
        }
        // 示例：创建发布深度控制指令的 Publisher
        // cmd_depth_pub_ = nh_.advertise<std_msgs::Float64>("cmd_depth", 1, true);
    }

    // 核心执行逻辑
    virtual void execute() override {
        double current_depth = NAN;
        if (variable_values_.count("DEPTH")) {
            current_depth = variable_values_["DEPTH"];
        }
        // 发布目标深度
        // std_msgs::Float64 msg; msg.data = target_depth_;
        // cmd_depth_pub_.publish(msg);

        // 如果深度误差小于阈值，则完成
        if (!std::isnan(current_depth) &&
            std::fabs(current_depth - target_depth_) <= completion_thresh_)
        {
            setCompleted();
        }
    }

protected:
    virtual void onActivate() override {
        ROS_INFO_STREAM("[" << instance_name_ << "] 开始保持深度 " << target_depth_);
    }
    virtual void onComplete() override {
        ROS_INFO_STREAM("[" << instance_name_ << "] 恒深完成");
    }
    virtual void onFailure() override {
        ROS_WARN_STREAM("[" << instance_name_ << "] 恒深失败 (超时或其他)");
    }

private:
    double target_depth_{0.0};
    double completion_thresh_{0.1};
    // ros::Publisher cmd_depth_pub_;
};

}  // namespace behavior_manager

#endif  // BEHAVIOR_MANAGER_CONSTANT_DEPTH_H
```

#### 2. 源文件：`ConstantDepth.cpp`
```cpp
#include "behavior_manager/ConstantDepth.h"
#include <pluginlib/class_list_macros.h>

namespace behavior_manager {

PLUGINLIB_EXPORT_CLASS(behavior_manager::ConstantDepth, behavior_manager::BehaviorBase)

}  // namespace behavior_manager
```

#### 3. YAML 配置（`behaviors.yaml`）
```yaml
vehicle_name: auv1

behaviors:
  - type: ConstantDepth
    name: constant_depth_1
    priority: 1
    pwt: 1.0
    duration: 10.0
    timeout_fails: false
    enable_debug: false
    conditions:
      - "COMPLETED:waypoint_1"
      - "DEPTH:>2.0"
    target_depth: 3.5
    completion_thresh: 0.05
```

> **流程说明**：
> 1. BehaviorManager 读取 `/behaviors/0`，得到 `type="ConstantDepth", name="constant_depth_1", ...` 并把其子树复制到 `/constant_depth_1`。  
> 2. 动态实例化 `behavior_manager::ConstantDepth("constant_depth_1")`，调用 `initialize(ros::NodeHandle("/constant_depth_1"))`。  
> 3. `initialize()` 从 `/constant_depth_1/priority` 等读取基类参数，并读取 `target_depth`、`completion_thresh`。同时基类会自动订阅 `/auv1/behavior_states/waypoint_1` 和 `/auv1/feedback/DEPTH`。  
> 4. BehaviorManager 调度时若 `checkCondition()` 返回 true，则调用 `activate()` → `run()` 循环，直到 `execute()` 调用 `setCompleted()` 或超时。  

---

### 带 Action 支持的中间类示例（`BehaviorWithAction`）

下面演示如何在需要**可抢占、带反馈与结果**的行为中使用 `BehaviorWithAction`。

#### 1. 头文件：`ObstacleAvoidBehavior.h`
```cpp
#ifndef BEHAVIOR_MANAGER_OBSTACLE_AVOID_BEHAVIOR_H
#define BEHAVIOR_MANAGER_OBSTACLE_AVOID_BEHAVIOR_H

#include "behavior_manager/BehaviorWithAction.h"
#include <your_pkg/ObstacleAvoidAction.h>  // 由 .action 文件生成

namespace behavior_manager {

class ObstacleAvoidBehavior
  : public BehaviorWithAction<your_pkg::ObstacleAvoidAction>
{
public:
    ObstacleAvoidBehavior(const std::string& name)
      : BehaviorWithAction<your_pkg::ObstacleAvoidAction>(name) {}

    virtual ~ObstacleAvoidBehavior() = default;

    // 当收到客户端的 Goal 请求时触发
    virtual void onGoal() override {
        auto goal = action_server_->acceptNewGoal();
        // 从 goal 中获取安全距离参数
        safe_distance_ = goal->safe_distance;
        // 激活行为
        activate();
    }

    // 当收到客户端的 Cancel 请求时触发
    virtual void onPreempt() override {
        action_server_->setPreempted();
        setFailed();  // 标记为失败，触发 onFailure()
    }

    // 核心执行逻辑
    virtual void execute() override {
        // 1) 从传感器读取障碍物距离
        // 2) 计算避障指令并发布
        // 3) 可以发布实时反馈
        //    your_pkg::ObstacleAvoidFeedback fb;
        //    action_server_->publishFeedback(fb);

        // 若检测到避障已完成
        if (checkObstacleCleared()) {
            your_pkg::ObstacleAvoidResult result;
            result.success = true;
            action_server_->setSucceeded(result);
            setCompleted();
        }
        // 否则保持 RUNNING
    }

protected:
    virtual bool checkConditionImpl() const override {
        // 只要收到一个有效的 Goal，就直接执行
        return true;
    }
    virtual void onActivate() override {
        ROS_INFO_STREAM("[" << instance_name_ << "] 避障激活，安全距离=" << safe_distance_);
    }
    virtual void onComplete() override {
        ROS_INFO_STREAM("[" << instance_name_ << "] 避障完成");
    }
    virtual void onFailure() override {
        ROS_WARN_STREAM("[" << instance_name_ << "] 避障失败/取消");
    }

private:
    bool checkObstacleCleared() {
        // 用户定义检测障碍物是否已清除的逻辑
        return false;
    }

    double safe_distance_{1.0};
};

}  // namespace behavior_manager

#endif  // BEHAVIOR_MANAGER_OBSTACLE_AVOID_BEHAVIOR_H
```

#### 2. 源文件：`ObstacleAvoidBehavior.cpp`
```cpp
#include "behavior_manager/ObstacleAvoidBehavior.h"
#include <pluginlib/class_list_macros.h>

namespace behavior_manager {

PLUGINLIB_EXPORT_CLASS(behavior_manager::ObstacleAvoidBehavior, 
                       behavior_manager::BehaviorBase)

}  // namespace behavior_manager
```

#### 3. YAML 配置（同样放在 `behaviors.yaml`）
```yaml
vehicle_name: auv1

behaviors:
  - type: ObstacleAvoid
    name: obstacle_avoid_1
    priority: 0
    pwt: 2.0
    duration: 15.0
    timeout_fails: true
    enable_debug: true
    conditions:
      - "RUNNING:constant_depth_1"
    safe_distance: 1.0
```

> **流程说明**：
> 1. 动态实例化 `ObstacleAvoidBehavior("obstacle_avoid_1")`。  
> 2. 当外部 Action 客户端向 `/auv1/obstacle_avoid_1/action` 发送 Goal 时，基类 `BehaviorWithAction` 会回调到 `onGoal()`，进而调用 `activate()`。  
> 3. `run()` 循环中执行 `execute()`，并利用 `action_server_->publishFeedback()` 发布反馈。  
> 4. 如果收到预empt，则调用 `onPreempt()`，并将行为标记为 `FAILED`，调用 `action_server_->setPreempted()`。  
> 5. 如果避障成功，调用 `setCompleted()` 并通过 `action_server_->setSucceeded()` 通知客户端。

---

## FAQ 常见问题

1. **为什么要让每个行为都有自己的 `ros::CallbackQueue` 与 `AsyncSpinner`？**  
   - 目的是「隔离不同行为的订阅回调」，避免在某个行为 `run()` 阶段阻塞时，无法及时处理“持续时长复位”或“条件订阅”导致逻辑失效。独立队列 + 异步 Spinner 确保监听与执行并行。

2. **`onTimeout()` 默认为什么把状态置为 `COMPLETED`？会影响逻辑吗？**  
   - 大多数“一次性行为”超时后并不算失败，而是“业务认为它已经结束”，故默认 `timeout_fails=false`，`onTimeout()` → `COMPLETED`。  
   - 如果某行为超时后要算作失败（如避障超时未躲避），可在 YAML 里 `timeout_fails: true`。也可由子类 override `onTimeout()` 自定义行为。

3. **什么时候需要覆盖 `checkConditionImpl()`？**  
   - 基类会自动解析 YAML 中的 `conditions`（状态条件与变量阈值），如果子类想要「完全自定义触发条件」（比如依赖某个自定义消息、某个复杂布尔表达式），就 override `checkConditionImpl()` 并返回自定义逻辑。  
   - 在这种情况下，基类的“YAML 条件”就不再生效（`hasOverrideCheckCondition_` 标记会切换到子类逻辑）。

4. **“自定义参数” 与 `VariableExtractor` 有何区别？**  
   - “自定义参数”是行为在初始化或执行时需要的常量配置，写在 YAML 里，直接通过 `nh_.param<…>` 一次性读取。  
   - `VariableExtractor` 则是动态订阅 `/vehicle_name/feedback/<VAR>` 等话题，在运行时持续更新“实时数值”，供条件触发判断（`<VAR>:<op><value>`）使用。这种数据通常与传感器反馈挂钩。

5. **在 YAML 中不写某些可选字段会怎样？**  
   - `priority` 不写，默认 `1`；  
   - `pwt` 不写，默认 `1.0`；  
   - `duration` 不写，默认 `0.0`（不超时）；  
   - `duration_idle_decay` 不写，默认为 `false`；  
   - `timeout_fails` 不写，默认为 `false`（超时即完成）；  
   - `enable_debug` 不写，默认为 `false`；  
   - `conditions` 不写，视为无需额外条件，直接允许激活；  
   - 任何“自定义参数”不写，子类读取时会用它们各自的默认值。

6. **如何知道有哪些行为类型可用？**  
   - 查看 `behavior_plugin.xml`，里面列出了所有 `type` 与对应的 `base_class_type`。例如：
     ```xml
     <library path="libbehavior_manager">
       <class type="behavior_manager::ConstantDepth"
              base_class_type="behavior_manager::BehaviorBase">
         <description>保持恒定深度</description>
       </class>
       <class type="behavior_manager::GoToDepth"
              base_class_type="behavior_manager::BehaviorBase">
         <description>前往指定深度</description>
       </class>
       <class type="behavior_manager::ObstacleAvoid"
              base_class_type="behavior_manager::BehaviorWithAction<your_pkg::ObstacleAvoidAction>">
         <description>避障行为</description>
       </class>
     </library>
     ```
   - 在 YAML 中 `type` 必须与此处 `type` 保持一致。

7. **如果行为需要额外订阅或发布话题，应如何处理？**  
   - 在子类 `initialize()` 里，自行创建对应的 `ros::Subscriber` 或 `ros::Publisher`，因为私有 `nh_` 已经命名到 `/instance_name`。若要订阅全局话题（如 `/auv1/imu`），可使用 `nh_parent_` 或显式 `ros::NodeHandle("/auv1")`。  
   - 例如：
     ```cpp
     imu_sub_ = nh_parent_.subscribe("/auv1/imu", 10, &MyBehavior::imuCallback, this);
     cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
     ```

8. **如何启用调试日志？**  
   - 在对应行为的 YAML 中，写 `enable_debug: true`。  
   - 通过 `rosconsole` 或运行时环境设置 `ROS_LOG_LEVEL` 至 `DEBUG`，即可看到基类大量 `ROS_DEBUG` 输出。

---

## 联系方式
- **项目维护者**：  
  - 姓名：XXX  
  - 邮箱：xxx@example.com  
  - 微信/QQ（如有）  

- **问题反馈**：  
  如果在使用过程中遇到 BUG、编译问题或功能扩展需求，欢迎提交 Issues 或发邮件到上述邮箱。  

---

**感谢使用 `BehaviorBase`，祝开发顺利！**
