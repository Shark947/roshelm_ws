# ROS × MOOS‑IvP 集成项目分析报告

## 1. 项目梳理与核心目标
本仓库围绕“将 MOOS‑IvP 的仲裁框架与原生水下行为模块逐步融入 ROS 生态”展开，核心目标包含两层：
1) **通信与数据融合**：把 MOOS‑IvP 的 `NAV_*` / `DESIRED_*` 机制在 ROS 中落地，实现可观察、可控、可复用的数据路径；
2) **决策能力迁移**：将 IvP‑Helm 的多目标决策能力从 MOOS 运行时逐步迁移至 ROS，最终用行为树替代 Helm 的任务编排能力，同时保留 IvP 的求解和基础行为。

对应形成三阶段演进路线：
- **阶段 1**：ROS/MOOS Bridge（ROS 与 MOOS‑IvP 并行运行）。
- **阶段 2**：ROS 化 Helm（只运行 ROS，读取 `.bhv` 行为文件）。
- **阶段 3**：ROS 行为树编排（用 BT 组织任务，但保留 IvP 求解器）。

## 2. 关键节点职责（与数据流相关）

### 2.1 传感/状态 → ROS 话题统一输出
- **variable_extractor_node**：启动后通过 pluginlib 自动发现并加载 `variable_extractor` 插件，读取 `vehicle_name` 参数，然后对插件声明的变量逐一订阅与发布，统一输出 `/[vehicle]/current_*` 话题。【F:src/variable_extractor/src/variable_extractor_node.cpp†L1-L54】
- **DefaultVariableExtractor 插件**：从 `/[vehicle]/pose_gt`、`/[vehicle]/dvl`、`/[vehicle]/imu` 中抽取位置、速度、姿态信息，发布为 `/[vehicle]/current_x|y|z|vx|vy|speed|yaw|pitch|roll` 等话题，同时为外部数据处理提供变量缓存（`var_map`）。【F:src/variable_extractor/include/variable_extractor/DefaultVariableExtractor.hpp†L14-L252】

**补充说明：**这一层的数据标准化是后续阶段复用的基础，后续无论是 MOOS bridge、ROS Helm 还是行为树方案，都通过 `/current_*` 话题完成与控制器及导航转换的衔接。【F:src/variable_extractor/include/variable_extractor/DefaultVariableExtractor.hpp†L14-L252】

### 2.2 ROS → NAV 话题转换
- **ros_helm::RosNavPublisher**：订阅 `/[vehicle]/current_vx|vy|yaw|z`，发布 `/[vehicle]/current_speed|heading|depth`。这是“ROS 风格导航变量”的生成器，作为 RosBridge 后续转成 `NAV_*` 的输入来源。【F:src/ros_helm/src/ros/src/RosNavPublisher.cpp†L1-L116】
- **ros_behavior_tree::nav_publisher_node**：在行为树方案中承担更完整的转换职责：
  - 从 `/current_vx/vy` 计算 `current_speed`，并进一步生成 `NAV_SPEED`；
  - 由 `/current_yaw/pitch/roll` 计算 `NAV_YAW/PITCH/ROLL` 和 `NAV_HEADING`；
  - 由 `/current_z` 生成 `current_depth` 与 `NAV_DEPTH`；
  - 同步发布 `NAV_X/NAV_Y` 用于任务航迹或路径约束。
  这些转换逻辑对齐 `ros_helm` 的姿态/航向转换策略，确保行为树与 Helm 的导航语义一致。【F:src/ros_behavior_tree/src/nav_publisher_node.cpp†L1-L352】

### 2.3 Helm/IvP 决策引擎（共用核心）
- **HelmIvP（Helm 应用层）**：MOOS‑IvP Helm 的核心应用，负责接收外部变量更新、维护 `InfoBuffer`、调度 `HelmEngine`、构建 `HelmReport` 并产生 `DESIRED_*` 输出。内部持有 `IvPDomain` 与 `HelmEngine` 指针，是运行期状态与行为集管理的中心。【F:src/ros_helm/src/helm/include/HelmIvP.h†L23-L171】
- **HelmEngine（求解层）**：IvP 求解引擎，接收 `BehaviorSet` 与 `InfoBuffer`，在迭代中完成“行为生成 IvPFunction → 约束检查 → 问题构建 → 求解 → 报告输出”的完整流程。【F:src/ros_helm/src/helm/include/HelmEngine.h†L23-L90】
- **IvPDomain（决策空间）**：维护每个决策维度（如 course/speed/depth）的离散范围与采样分辨率，是 IvP 求解的空间定义基础。【F:src/ros_helm/src/ivp/src/lib_ivpcore/IvPDomain.h†L21-L111】
- **IvPFunction（目标函数）**：由每个行为生成的目标函数实体，HelmEngine 汇总多个 IvPFunction 并求解最优组合，最终转化为决策变量上的 `DESIRED_*` 输出。【F:src/ros_helm/src/ivp/src/lib_ivpcore/IvPFunction.h†L23-L61】

### 2.4 输出 → 控制
- **RosBridge（ros_helm）**：
  - **输入侧**：订阅 `/current_*`（含位置、姿态、速度、深度等）并转化为 `NAV_*`，注入 Helm 作为决策输入。
  - **输出侧**：从 Helm 读取 `DESIRED_*` 并发布 `/[vehicle]/desired_heading|speed|depth` 话题，供控制器使用。
  - **额外职责**：维护状态日志与模式状态发布，保证 Helm 与 ROS 的状态可观测性。【F:src/ros_helm/src/ros/src/RosBridge.cpp†L37-L214】【F:src/ros_helm/src/ros/src/RosBridge.cpp†L407-L509】
- **controller_manager_node**：读取 YAML 配置加载控制器插件，根据 `desired_*` 与 `current_*` 计算控制输出并发布 `/[vehicle]/output_*`，形成闭环控制通道。【F:src/controller_manager/src/controller_manager_node.cpp†L1-L37】【F:src/controller_manager/src/controller_manager.cpp†L44-L140】

### 2.5 ROS/MOOS Bridge（阶段 1）
- **RosMoosBridge**：在 ROS/MOOS 共存阶段承担双向转换：
  - **ROS → MOOS**：订阅 `/pose_gt`、`/imu`、`/dvl`，生成 `NAV_*` 并通过 MOOS 通信发送给 MOOSDB；
  - **MOOS → ROS**：从 MOOS 接收 `DESIRED_*` 并发布 `/desired_*`，作为控制器的目标输入。
  该节点是阶段 1 的关键集成点，也是 MOOS‑IvP 与 ROS 的唯一数据通道。【F:src/ros_moos_bridge/src/RosMoosBridge.cpp†L17-L119】

## 3. 三阶段演进与复用关系（逐阶段对比）

### 阶段 1：ROS/MOOS Bridge（基线）
**数据流（核心链路）**
1. 传感器 → ROS：`pose_gt / imu / dvl` 进入 ROS，提供位姿与速度测量。 
2. ROS → MOOS：`RosMoosBridge` 将 ROS 的位置/姿态/速度转换为 `NAV_*` 并 `Notify` 到 MOOSDB，形成 MOOS 侧决策输入。【F:src/ros_moos_bridge/src/RosMoosBridge.cpp†L33-L84】
3. MOOS Helm（pHelmIvP）计算决策输出 `DESIRED_*`。 
4. MOOS → ROS：Bridge 将 `DESIRED_*` 发布到 ROS `/desired_*` 话题。【F:src/ros_moos_bridge/src/RosMoosBridge.cpp†L86-L112】
5. ROS 控制：`controller_manager` 订阅 `/desired_*` 与 `/current_*`，输出 `/output_*` 驱动执行层。【F:src/controller_manager/src/controller_manager.cpp†L70-L140】

**节点职责与复用关系**
- ROS 侧只承担传感数据采集与桥接；决策/仲裁完全在 MOOS‑IvP 中完成。
- `RosMoosBridge` 是唯一数据通道：负责 ROS ↔ MOOS 的 NAV/DESIRED 双向映射。

### 阶段 2：ROS 化 Helm（对比阶段 1）
**改进点（相较阶段 1）**
- **移除对 MOOSDB 的运行依赖**：`HelmIvP` 直接在 ROS 节点中实例化与循环运行，MOOS‑IvP 的 Helm 逻辑与 IvP 求解器被“内嵌”到 ROS 节点中。【F:src/ros_helm/src/ros/src/ros_node_main.cpp†L1-L56】
- **复用 IvP 核心**：仍使用 `HelmIvP + HelmEngine + IvPDomain/IvPFunction` 的决策链路，保留 `.bhv` 行为文件定义方式。【F:src/ros_helm/src/helm/include/HelmIvP.h†L23-L171】【F:src/ros_helm/src/helm/include/HelmEngine.h†L23-L90】

**数据流（核心链路）**
1. 传感器 → `variable_extractor`：传感数据转成 `/current_*` 话题。【F:src/variable_extractor/include/variable_extractor/DefaultVariableExtractor.hpp†L14-L252】
2. ROS → NAV：
   - `RosNavPublisher` 计算 `current_speed/heading/depth`；
   - `RosBridge` 订阅 `current_*` 并生成 `NAV_*`，同时进行时间戳对齐、坐标转换与必要的派生量计算（如 NAV_SPEED 与 NAV_HEADING）。【F:src/ros_helm/src/ros/src/RosNavPublisher.cpp†L1-L116】【F:src/ros_helm/src/ros/src/RosBridge.cpp†L37-L214】
3. Helm 求解：`HelmIvP` 在 ROS 进程内运行，调用 `HelmEngine` 对行为集求解，形成 `DESIRED_*`。【F:src/ros_helm/src/helm/include/HelmIvP.h†L23-L171】【F:src/ros_helm/src/helm/include/HelmEngine.h†L23-L90】
4. DESIRED → 控制：`RosBridge` 发布 `/desired_*`，`controller_manager` 消费并产生 `/output_*`。【F:src/ros_helm/src/ros/src/RosBridge.cpp†L83-L200】【F:src/controller_manager/src/controller_manager.cpp†L70-L140】

**与阶段 1 的对比总结**
- **架构上**：MOOSDB 与 pHelmIvP 进程移除，决策环路缩短为“ROS 话题 → HelmIvP → ROS 话题”。
- **复用上**：IvP 求解器与行为库保持不变，仅改变上层运行环境与通信机制。
- **维护上**：减少跨系统依赖，调试边界收敛为 ROS 节点与话题。

### 阶段 3：行为树编排（对比阶段 2）
**改进点（相较阶段 2）**
- **任务编排方式变化**：由 `.bhv` 行为文件 → 行为树 XML；任务阶段切换由 BT 节点控制，更易扩展与可视化。
- **复用 IvP 求解核心**：`HelmInterface` 内部仍创建 `HelmEngine` 与 `BehaviorSet`，并使用 `IvPDomain` 配置决策空间。【F:src/ros_behavior_tree/src/helm_interface.cpp†L96-L180】

**数据流（核心链路）**
1. 传感器 → `variable_extractor`：产生 `/current_*` 话题（同阶段 2）。【F:src/variable_extractor/include/variable_extractor/DefaultVariableExtractor.hpp†L14-L252】
2. ROS → NAV：`nav_publisher_node` 计算并发布 `current_speed/heading/depth` 与 `NAV_*`，作为行为树与 IvP 求解的输入。【F:src/ros_behavior_tree/src/nav_publisher_node.cpp†L1-L352】
3. BT → HelmInterface：
   - `NavDataSubscriber` 同时订阅 ROS 与 NAV 风格话题，将数据写入 `NavDataStore`，并同步 DEPLOY/RETURN/SPD 触发信息。【F:src/ros_behavior_tree/src/nav_subscriber.cpp†L12-L119】
   - `HelmInterface` 从 `NavDataStore` 拉取首选导航数据，写入 `InfoBuffer` 并调用 `HelmEngine`，生成 `HelmReport` 与 `DESIRED_*` 发布。【F:src/ros_behavior_tree/src/helm_interface.cpp†L180-L330】
4. DESIRED → 控制：同样由 `controller_manager` 消费 `/desired_*`，输出 `/output_*`。【F:src/controller_manager/src/controller_manager.cpp†L70-L140】

**与阶段 2 的对比总结**
- **行为组织方式升级**：行为树对任务阶段、模式切换、并行/顺序关系描述更直接。
- **求解内核保持一致**：HelmEngine 与 IvPFunction 仍作为行为融合与求解核心。
- **数据路径更可控**：`NavDataStore → InfoBuffer → HelmEngine` 显式化，便于诊断数据质量和行为效果。

## 4. 典型数据流总结（按“从 ROS 话题到控制输出”）

### 4.1 ROS/MOOS Bridge（阶段 1）
```
ROS 传感话题 (pose_gt/imu/dvl)
  → RosMoosBridge: NAV_* Notify
  → MOOSDB / pHelmIvP
  → DESIRED_* (MOOS)
  → RosMoosBridge: /desired_* (ROS)
  → controller_manager: /output_*
```
（ROS → MOOS → ROS 形成闭环，桥接节点承担 NAV/ DESIRED 转换）【F:src/ros_moos_bridge/src/RosMoosBridge.cpp†L33-L112】【F:src/controller_manager/src/controller_manager.cpp†L70-L140】

### 4.2 ROS 化 Helm（阶段 2）
```
传感器 → variable_extractor → /current_*
  → RosNavPublisher 生成 /current_speed|heading|depth
  → RosBridge 聚合 NAV_* 并注入 HelmIvP
  → HelmIvP + HelmEngine + IvPDomain/Function
  → RosBridge 发布 /desired_*
  → controller_manager 输出 /output_*
```
（Helm 内嵌 ROS 进程，保留 IvP 求解链路）【F:src/variable_extractor/include/variable_extractor/DefaultVariableExtractor.hpp†L14-L252】【F:src/ros_helm/src/ros/src/RosNavPublisher.cpp†L1-L116】【F:src/ros_helm/src/ros/src/RosBridge.cpp†L37-L214】【F:src/ros_helm/src/helm/include/HelmIvP.h†L23-L171】【F:src/controller_manager/src/controller_manager.cpp†L70-L140】

### 4.3 行为树编排（阶段 3）
```
传感器 → variable_extractor → /current_*
  → nav_publisher_node 生成 /current_speed|heading|depth + /NAV_*
  → NavDataSubscriber → NavDataStore
  → HelmInterface: InfoBuffer + HelmEngine 求解
  → /desired_*
  → controller_manager 输出 /output_*
```
（行为树负责任务组织；IvP 求解器仍用于融合行为输出）【F:src/variable_extractor/include/variable_extractor/DefaultVariableExtractor.hpp†L14-L252】【F:src/ros_behavior_tree/src/nav_publisher_node.cpp†L1-L352】【F:src/ros_behavior_tree/src/nav_subscriber.cpp†L12-L119】【F:src/ros_behavior_tree/src/helm_interface.cpp†L180-L330】【F:src/controller_manager/src/controller_manager.cpp†L70-L140】

## 5. 复用内容总结
- **IvP 求解内核复用**：`HelmEngine`、`IvPDomain`、`IvPFunction` 在阶段 2 与阶段 3 中均为核心决策引擎。【F:src/ros_helm/src/helm/include/HelmEngine.h†L23-L90】【F:src/ros_helm/src/ivp/src/lib_ivpcore/IvPDomain.h†L21-L111】【F:src/ros_helm/src/ivp/src/lib_ivpcore/IvPFunction.h†L23-L61】
- **ROS 话题标准化复用**：`/current_*`（状态）、`/NAV_*`（MOOS 风格）、`/desired_*`（目标）与 `/output_*`（控制输出）在各阶段保持一致，便于控制器与上层算法复用。【F:src/variable_extractor/include/variable_extractor/DefaultVariableExtractor.hpp†L14-L252】【F:src/ros_helm/src/ros/src/RosBridge.cpp†L83-L200】【F:src/controller_manager/src/controller_manager.cpp†L70-L140】

## 6. 关键路径的更细粒度说明

### 6.1 ros_helm 主循环与数据衔接
`ros_helm` 的主循环在 ROS 节点内完成 Helm 调度，关键步骤如下：
1) `RosBridge::deliverPending()` 把 ROS 输入（NAV / 命令话题）封装为 HelmMail 并送入 `HelmIvP::OnNewMail()`；
2) `HelmIvP::Iterate()` 驱动 IvP 行为集与求解引擎输出 `DESIRED_*`；
3) `RosBridge::publishDesired()` 把 Helm 输出转换为 `/desired_*` 话题供控制器使用；
4) `RosBridge::publishModeState()` 与 `logStatusIfNeeded()` 负责模式与日志输出。
该流程在 `ros_node_main.cpp` 中循环执行，形成“ROS 话题 → Helm → ROS 话题”的闭环。【F:src/ros_helm/src/ros/src/ros_node_main.cpp†L1-L56】【F:src/ros_helm/src/ros/src/RosBridge.cpp†L331-L509】

### 6.2 ros_behavior_tree 的导航订阅与求解入口
行为树方案的入口由 `NavDataSubscriber` 负责，它同时订阅 ROS 风格与 NAV 风格话题，把数据写入 `NavDataStore`。随后 `HelmInterface::updateInfoBuffer()` 在每次 BT tick 时把最新导航数据写入 `InfoBuffer`，作为 `HelmEngine` 求解输入。
这使得 BT 可以只关注任务逻辑，IvP 继续做速度/航向/深度决策融合。【F:src/ros_behavior_tree/src/nav_subscriber.cpp†L12-L119】【F:src/ros_behavior_tree/src/helm_interface.cpp†L180-L330】

### 6.3 controller_manager 的闭环控制与话题约定
`controller_manager` 在初始化时自动拼接 `/[vehicle]/desired_*` 与 `/[vehicle]/current_*` 话题，并输出 `/[vehicle]/output_*`，保持与 Helm/BT 输出规范一致。它在收到 `current_*` 时做控制更新，因此 DESIRED 的刷新频率与当前状态更新频率会直接影响控制响应性能。【F:src/controller_manager/src/controller_manager.cpp†L70-L140】

---

> 如需进一步把 docking_nav / docking_optical 的实验流程、行为树 XML 示例或具体 `.bhv` 行为文件纳入分析，可在后续补充以任务视角展开。
