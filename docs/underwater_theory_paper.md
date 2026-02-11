# 水下理论：面向完全浸没水下体的环境实现、运动学与水动力学统一框架（基于 uuv_simulator）

## 摘要
本文面向“完全浸没（fully submerged）”水下体，对 `uuv_simulator` 的环境建模实现、运动学描述与水动力学建模进行统一梳理。研究对象限定为不涉及自由液面航行体（surface vessel）的小角度稳性模型，聚焦水下航行器在恒定/随机海流中的六自由度动力学。实现层面上，系统保留 Gazebo/ODE 刚体引擎，并通过世界插件生成海流场、通过模型插件注入附加质量、阻尼、附加科里奥利与浮力项，以“外力叠加”方式完成从空气/陆地刚体仿真到水下环境仿真的扩展。本文给出离散海流随机过程公式、球坐标流速重构公式、六自由度运动学与动力学方程、以及与代码实现的一一对应关系，并讨论参数可辨识性与坐标系一致性问题。

**关键词**：完全浸没；水下理论；六自由度运动学；水动力学；Gauss-Markov 海流；uuv_simulator

---

## 1. 引言

在工程仿真中，水下物理并非通过替换底层刚体求解器实现，而是通过在刚体动力学方程右端增添“流体作用广义力”实现。`uuv_simulator` 采用该路线：

1. 世界层（World Plugin）生成海流速度场；
2. 载体层（Model Plugin）按每个时间步计算并施加水动力与浮力；
3. Gazebo/ODE 继续负责刚体积分、关节约束与碰撞求解。

这种结构具有模块化、可标定、可在线调参的优点，适用于科研与工程迭代。

---

## 2. 系统架构与问题定义

### 2.1 研究对象与假设

本文仅考虑完全浸没水下体：

- 航行器主体始终位于水面以下；
- 不使用自由液面船舶的稳性附加项（metacentric 宽度/长度模型）；
- 主要流体作用由以下项构成：
  \[
  \tau_{hydro}=\tau_D+\tau_A+\tau_C,
  \]
  分别对应阻尼、附加质量、附加科里奥利。

### 2.2 仿真架构

世界文件定义 ODE 物理与海流插件，海流插件发布 `hydrodynamics/current_velocity`；模型插件订阅该流速并为各 link 构建水动力模型，在更新循环中施加水动力与浮力。该架构形成“环境场→局部相对流速→流体力→刚体积分”的闭环。

---

## 3. 环境实现：海流场建模与发布

### 3.1 海流状态变量

海流由三个随机过程描述：

- 流速模长 \(V(t)\)；
- 水平角 \(\alpha(t)\)；
- 垂直角 \(\beta(t)\)。

### 3.2 离散随机过程（代码实现）

`GaussMarkovProcess` 的离散更新可写为
\[
 x_{k+1}=(1-\mu\Delta t)x_k+\eta_k,
\]
其中 \(\eta_k\sim\mathcal N(0,\sigma^2)\)，\(\sigma=\texttt{noiseAmp}\)，并进行区间截断
\[
 x_{k+1}\in[x_{\min},x_{\max}].
\]

该实现是一阶衰减+高斯噪声+限幅模型，适用于海流强度与方向的平稳随机扰动仿真。

### 3.3 流速向量重构

插件按下式将 \((V,\alpha,\beta)\) 映射为三维流速：
\[
\mathbf v_c=
\begin{bmatrix}
V\cos\alpha\cos\beta\\
V\sin\alpha\cos\beta\\
V\sin\beta
\end{bmatrix}.
\]

随后世界插件发布该向量至海流话题，供所有水下体共享（或由各体局部覆盖）。

### 3.4 工程解释

- \(\mu\) 越大，状态回落更快；
- \(\sigma\) 越大，短时扰动更强；
- \([x_{\min},x_{\max}]\) 控制海况可达范围。

---

## 4. 运动学理论：完全浸没水下体六自由度描述

尽管插件主要实现动力学外力注入，完整“水下理论”仍需以六自由度运动学作为上层状态演化基础。

### 4.1 状态变量

设地固系位姿
\[
\eta=[x,\ y,\ z,\ \phi,\ \theta,\ \psi]^\top,
\]
体坐标速度
\[
\nu=[u,\ v,\ w,\ p,\ q,\ r]^\top.
\]

### 4.2 运动学映射

标准六自由度运动学写为
\[
\dot\eta = J(\eta)\nu,
\]
其中
\[
J(\eta)=\begin{bmatrix}
R(\phi,\theta,\psi)&0\\
0&T(\phi,\theta)
\end{bmatrix}.
\]

- \(R\) 为姿态旋转矩阵（体坐标到地固系）；
- \(T\) 为角速度到欧拉角速度映射矩阵。

在数值实现中，Gazebo 维护刚体位姿/速度积分；插件读取 link 速度并构造相对流速，不直接手写积分器。

---

## 5. 水动力学理论与实现映射

### 5.0 水体静力学：完全浸没水下体的浮力与恢复力矩（详细公式）

本节先给出静力学项 \(g(\eta)\) 的严格表达，再进入 Fossen 动力学项。设：

- 地固系（惯性系）单位竖直向上向量 \(e_3=[0,0,1]^\top\)；
- 体坐标系原点取于质心 \(G\)；
- 浮心为 \(B\)，其在体坐标系中的位置向量记为
  \(r_B=[x_B,y_B,z_B]^\top\)；
- 重力加速度为 \(g\)，流体密度为 \(\rho\)，排水体积为 \(\nabla\)（常用符号 Delta 或 \(V\)）。

对完全浸没且流体近似不可压、\(\rho\) 常数的刚体：

1. **重力（在地固系）**
\[
\mathbf f_g^n = \begin{bmatrix}0\\0\\-W\end{bmatrix},\qquad W=mg.
\]

2. **浮力（在地固系）**
\[
\mathbf f_b^n = \begin{bmatrix}0\\0\\B\end{bmatrix},\qquad B=\rho g\nabla.
\]

3. **转到体坐标系**（\(R_{nb}=R^\top\)，即地固到体坐标）
\[
\mathbf f_g^b = R_{nb}\,\mathbf f_g^n,
\qquad
\mathbf f_b^b = R_{nb}\,\mathbf f_b^n.
\]

采用 ZYX 欧拉角 \((\phi,\theta,\psi)\) 且忽略 \(\psi\) 对竖直投影的影响，可得常用显式形式：
\[
\mathbf f_g^b=
\begin{bmatrix}
-W\sin\theta\\
W\cos\theta\sin\phi\\
W\cos\theta\cos\phi
\end{bmatrix},
\qquad
\mathbf f_b^b=
\begin{bmatrix}
B\sin\theta\\
-B\cos\theta\sin\phi\\
-B\cos\theta\cos\phi
\end{bmatrix}.
\]

4. **静力恢复力矩（绕质心）**
\[
\boldsymbol\tau_g^b = r_G\times \mathbf f_g^b,
\qquad
\boldsymbol\tau_b^b = r_B\times \mathbf f_b^b.
\]
若体坐标原点取质心，通常 \(r_G=0\)，则重力力矩为 0，仅保留浮力偏置力矩。

5. **六维静力项**
\[
g(\eta)=
\begin{bmatrix}
\mathbf f_g^b+\mathbf f_b^b\\
\boldsymbol\tau_g^b+\boldsymbol\tau_b^b
\end{bmatrix}.
\]

6. **中性浮力条件**
\[
W=B\iff m=\rho\nabla.
\]
当 \(W=B\) 且 \(r_B\neq r_G\) 时，净垂向力为 0，但仍可能存在姿态恢复力矩。

> 对应到 `uuv_simulator`，完全浸没时浮力主项即 \(\rho g\nabla\)，并通过在浮心处施力生成恢复力矩；中性浮力由 `volume = mass/fluidDensity` 实现。

### 5.1 相对流速定义

模型插件将世界系海流速度旋转到体坐标并构造相对速度：
\[
\nu_r = \nu - \nu_c,
\]
其中 \(\nu_c\) 来自世界插件发布的 \(\mathbf v_c\)（角速度项常取 0）。

### 5.2 阻尼项

在实现中，阻尼由线性项、前向速度相关项与非线性项组合，可写为
\[
\tau_D=-D(\nu_r)\nu_r.
\]

其矩阵结构可概括为
\[
D(\nu_r)=
\Big(D_{lin}+\delta_{lin}I\Big)
+u\Big(D_u+\delta_u I\Big)
+\operatorname{diag}\big((d_{nl,i}+\delta_{nl})|\nu_{r,i}|\big),
\]
再乘尺度系数 \(s_D\)。

### 5.3 附加质量项

附加质量作用写为
\[
\tau_A=-M_A\dot\nu_r,
\]
并支持尺度与偏置：
\[
M_A^*=s_A(M_A+\delta_A I).
\]

由于仿真引擎加速度读数在部分版本存在噪声/相位问题，实现中采用速度差分+一阶滤波估计 \(\dot\nu_r\)。

### 5.4 附加科里奥利项

基于 Fossen 形式：
\[
\tau_C=-C_A(\nu_r)\nu_r,
\]
其中 \(C_A\) 由 \(M_A\) 与 \(\nu_r\) 构造反对称分块矩阵。

### 5.5 浮力项（完全浸没场景）

完全浸没条件下，浮力大小近似恒定：
\[
\mathbf F_b =
\begin{bmatrix}
0\\0\\\rho g V
\end{bmatrix}.
\]

若浮心与质心不重合，则存在恢复力矩
\[
\tau_b = r_{GB}\times F_b,
\]
实现中通过 `AddForceAtRelativePosition` 在浮心施加浮力，自动形成力矩。

### 5.6 总体动力学方程

综合上述项，可写为
\[
M\dot\nu + C(\nu)\nu + D(\nu_r)\nu_r + g(\eta)
=\tau_{ctrl}+\tau_{dist},
\]
其中在 `uuv_simulator` 插件侧显式注入的是
\[
\tau_{hydro}=\tau_D+\tau_A+\tau_C+\tau_b.
\]

### 5.7 Fossen 六自由度模型的完整写法与逐项公式

#### 5.7.1 标准紧凑形式

Fossen 6-DoF 刚体-流体耦合动力学常写为
\[
\underbrace{M}_{M_{RB}+M_A}\dot\nu+
\underbrace{C(\nu)}_{C_{RB}(\nu)+C_A(\nu_r)}\nu+
\underbrace{D(\nu_r)}_{D_{lin}+D_{nl}(\nu_r)}\nu_r+
g(\eta)=\tau.
\]

在海流存在时，更常见的实现形式是
\[
M_{RB}\dot\nu + C_{RB}(\nu)\nu + M_A\dot\nu_r + C_A(\nu_r)\nu_r + D(\nu_r)\nu_r + g(\eta)=\tau.
\]

#### 5.7.2 质量矩阵项

1) 刚体质量矩阵（以体坐标原点位于质心为例）
\[
M_{RB}=\begin{bmatrix}
mI_3 & 0\\
0 & I_G
\end{bmatrix},
\]
其中 \(I_G\) 为绕质心惯性张量。

若体坐标原点不在质心，令 \(r_G=[x_G,y_G,z_G]^\top\)、
\(S(r_G)\) 为反对称算子（满足 \(S(a)b=a\times b\)），则
\[
M_{RB}=\begin{bmatrix}
mI_3 & -mS(r_G)\\
mS(r_G) & I_O
\end{bmatrix}.
\]

2) 附加质量矩阵
\[
M_A=\begin{bmatrix}
A_{11} & A_{12}\\
A_{21} & A_{22}
\end{bmatrix}\in\mathbb R^{6\times 6},
\]
工程上由辨识、经验或 CFD 给出；`uuv_simulator` 支持全矩阵/对角项、缩放与偏置。

#### 5.7.3 科里奥利与离心项

1) 刚体项
\[
C_{RB}(\nu)=
\begin{bmatrix}
0_{3\times3} & -mS(v)-mS(\omega)S(r_G)\\
-mS(v)+mS(r_G)S(\omega) & -S(I_O\omega)
\end{bmatrix},
\]
其中 \(\nu=[v^\top,\omega^\top]^\top\)、\(v=[u,v,w]^\top\)、\(\omega=[p,q,r]^\top\)。

当原点位于质心（\(r_G=0\)）时简化为
\[
C_{RB}(\nu)=
\begin{bmatrix}
0 & -mS(v)\\
-mS(v) & -S(I_G\omega)
\end{bmatrix}.
\]

2) 附加质量项
设
\[
a=M_A\nu_r=\begin{bmatrix}a_1\\a_2\end{bmatrix},
\]
则常用表达
\[
C_A(\nu_r)=
\begin{bmatrix}
0 & -S(a_1)\\
-S(a_1) & -S(a_2)
\end{bmatrix}.
\]

#### 5.7.4 阻尼项

阻尼力一般写为
\[
\tau_D=-D(\nu_r)\nu_r,
\]
其中
\[
D(\nu_r)=D_{lin}+D_q(\nu_r),
\qquad
D_q(\nu_r)=\operatorname{diag}(d_i|\nu_{r,i}|).
\]

在 `uuv_simulator` 中还包含前向速度相关线性阻尼与多类 offset/scaling：
\[
D = s_D\left[
\left(D_{lin}+\delta_{lin}I\right)
+u\left(D_u+\delta_uI\right)
+\operatorname{diag}\left((d_{nl,i}+\delta_{nl})|\nu_{r,i}|\right)
\right].
\]

#### 5.7.5 静力恢复项

由第 5.0 节：
\[
g(\eta)=
\begin{bmatrix}
\mathbf f_g^b+\mathbf f_b^b\\
r_G\times \mathbf f_g^b + r_B\times \mathbf f_b^b
\end{bmatrix}.
\]

完全浸没且中性浮力时（\(W=B\)）：
\[
\mathbf f_g^b+\mathbf f_b^b=0,
\]
但当 \(r_B\neq r_G\) 时仍有
\[
\tau_{rest}=r_B\times \mathbf f_b^b.
\]

#### 5.7.6 相对速度与海流耦合

将海流速度转入体坐标后，定义
\[
\nu_r = \nu-\nu_c.
\]
若仅考虑平移海流，可令
\[
\nu_c=[u_c,v_c,w_c,0,0,0]^\top.
\]

因此 Fossen 方程中所有流体相关项（\(M_A,C_A,D\)）均作用于 \(\nu_r\) 或 \(\dot\nu_r\)。

---

## 6. 从“环境实现”到“运动-动力统一”的计算流程

单步计算可归纳为：

1. 世界插件更新海流随机过程 \((V,\alpha,\beta)\) 并发布 \(\mathbf v_c\)；
2. 模型插件读取 \(\mathbf v_c\)，转换到体坐标；
3. 构造 \(\nu_r\)、估计 \(\dot\nu_r\)；
4. 计算 \(\tau_D,\tau_A,\tau_C\)；
5. 计算并施加完全浸没浮力 \(\tau_b\)；
6. 将总流体广义力施加到 link，交由 Gazebo 刚体引擎完成状态积分。

该流程保证“环境扰动—动力响应—位姿演化”一致闭环。

---

## 7. 坐标系与一致性问题（工程重点）

实现中存在世界流场方向与海洋动力学惯用坐标约定差异。插件日志已显式提示当前速度在 ENU 框架计算；而水动力计算内部进行了 NED/ENU 变换。因此在参数辨识与对比试验中，必须统一以下对象：

- 世界流速话题向量定义；
- 体坐标速度定义；
- 方程推导所用符号约定（NED/ENU）。

若忽略该一致性，常出现“阻尼方向正确但附加项符号错位”的现象。

---

## 8. 结论

针对完全浸没水下体，`uuv_simulator` 的“水下理论”可以归纳为：

1. **环境层**：以可配置随机过程构造海流场；
2. **动力层**：按 Fossen 结构计算阻尼、附加质量、附加科里奥利与浮力；
3. **积分层**：依托 Gazebo 刚体引擎进行位姿/速度演化。

该方法兼顾了工程可实现性与理论可解释性，适合用于水下航行器建模、参数敏感性分析与控制器仿真前验证。

---

## 附：符号表

- \(\eta\)：地固系位姿向量；
- \(\nu\)：体坐标速度向量；
- \(\nu_r\)：相对流体速度；
- \(M_A\)：附加质量矩阵；
- \(C_A\)：附加科里奥利矩阵；
- \(D\)：阻尼矩阵；
- \(\rho\)：流体密度；
- \(V\)：排水体积；
- \(\mu,\sigma\)：海流随机过程参数。

---

## 参考文献

[1] Fossen, T. I. *Handbook of Marine Craft Hydrodynamics and Motion Control*. Wiley, 2011.  
[2] UUV Simulator source code and plugin implementation (`uuv_world_plugins`, `uuv_gazebo_plugins`).
