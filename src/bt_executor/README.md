# bt_executor

`bt_executor` 是一个基于 BehaviorTree.CPP 的对接（docking）任务调度器，用于在不重写既有 MOOS-IvP 行为的前提下，通过行为树来显式表达任务阶段与切换条件。

它与原有的 `ros_helm + docking_nav` 路径并行存在：

- `auh_docking.launch` 继续保持原路径（作为回退/对照）。
- `auh_docking_bt.launch` 提供 BT 驱动的入口（opt-in）。

## 架构概览

在 BT 驱动模式下，系统的职责划分如下：

1. **Behavior Tree 负责“何时激活哪个行为”**。
2. **MOOS-IvP 行为负责“如何生成决策变量”**（由 `HelmEngine` 求解）。
3. **DockingPhaseManager 负责阶段推进、重试与回退逻辑**。

核心组件：

- `BTExecutor`：加载参数、构建黑板、注册节点并驱动行为树 tick，同时协调 helm 求解与指令发布。
- `HelmAdapter`：构建 `IvPDomain`，实例化 `lib_behaviors-marine` 行为，管理 `InfoBuffer/BehaviorSet/HelmEngine`，并对外提供行为激活与求解接口。
- `DockingPhaseManager`：从导航/视觉反馈推导 docking 阶段，并向 helm/info buffer 注入阶段相关的目标值与标志。
- BT 节点（`nodes/actions.*`, `nodes/conditions.*`）：把阶段管理、行为激活、flag 检查等逻辑变成显式节点。

## 运行方式

### 1) 直接启动 bt_executor

使用包内 launch：

```bash
roslaunch bt_executor docking_bt.launch
```

该 launch 会：

- 读取参数文件 `config/docking_bt.yaml`；
- 将其加载到命名空间 `bt_executor_node/`；
- 启动节点 `bt_executor_node`。

参见：`launch/docking_bt.launch`。

### 2) 在完整仿真中启动 BT 版本 docking

使用 AUH 的 BT 入口：

```bash
roslaunch auh_launch auh_docking_bt.launch
```

该 launch 在原仿真链路基础上，将最后一步替换为：

```xml
<include file="$(find bt_executor)/launch/docking_bt.launch"/>
```

参见：`auh_launch/launch/auh_docking_bt.launch`。

## 配置说明（config/docking_bt.yaml）

所有 BT 调度与 helm/行为参数集中在：

- `config/docking_bt.yaml`

建议关注以下分组：

- `tree/*`：行为树文件路径与 tick 频率；
- `topics/*`：输入/输出 topic 约定；
- `helm/*`：IvP domain、初值、行为定义与参数；
- `docking/*`：阶段推进、重试、深度/朝向策略相关参数；
- `commands/*`：MODE、flag、深度更新等桥接发布策略。

### 常见改动入口

1. **替换/调整行为树**

修改：

- `behavior_trees/docking.xml`
- 或在 YAML 中覆盖 `tree/path`。

2. **调行为参数（如 pwt/capture/runflag/endflag）**

修改：

- `helm/behaviors` 列表中的各行为参数。

3. **调阶段推进策略（对齐深度、重试阈值等）**

修改：

- `docking/*` 相关参数。

## 主要输入输出

尽管具体 topic 名称可在 YAML 中调整，但从职责上可分为：

- 输入：导航状态、任务模式（MODE）、视觉反馈/flag；
- 输出：控制相关决策变量（来自 helm 求解）、以及 docking 过程观测输出（phase/optical feedback）。

与 docking 过程可观测性直接相关的发布器位于：

- `include/bt_executor/docking_outputs_publisher.hpp`
- `src/docking_outputs_publisher` 的调用点在 `DockingPhaseUpdate` 条件节点中。

## 开发与调试建议

1. **优先通过 YAML 调参**，避免把任务策略硬编码进行为文件。
2. **把“切换条件”放进 BT 条件节点**，把“控制生成”留给 MOOS-IvP 行为。
3. 调试阶段推进问题时：
   - 重点查看 `DockingPhaseManager` 的日志输出；
   - 同时观察 `/docking/phase` 与视觉反馈 topic 是否一致。