# TwinCAT 电子凸轮双边切割仿真设计

## 1. 目标

在现有 TwinCAT 3.1.4024.75 工程中实现一个基于动态 Motion Function 凸轮表的电子凸轮示例：

- Axis 1 是传送带主轴，恒速运行。
- Axis 2 是线性往返切割从轴。
- 传送带上的四边形物体按 500 mm 节距周期出现。
- 只模拟四边形的两条边。
- 第一条边切割时，从轴速度为传送带速度加相对切割速度。
- 第二条边切割时，从轴速度为传送带速度减相对切割速度。
- 第二条边完成后，从轴在下一个物体到达前返回绝对准备位置。
- 同一套 PLC 代码用于仿真和实机，二者通过 TwinCAT NC 工程配置切换。
- 不实现 Visualization；通过在线变量和 TwinCAT Scope 验证。

## 2. 已有工程约束

- 工程版本：TwinCAT 3.1.4024.75。
- PLC 已引用 `Tc2_MC2` 和 `Tc2_MC2_Camming`。
- 系统工程已有两个 NC 轴和两套实际 EtherCAT 驱动映射。
- PLC 的 `MAIN` 当前为空。
- `cam_case.tsproj` 当前存在用户未提交修改，实施时不得覆盖或顺带提交该修改。
- `Tc2_MC2_Camming` 运行时需要相应的 TF5050 NC Camming 许可；仿真调试可使用可用的试用许可。

## 3. 范围

### 3.1 包含

- 分层的 Structured Text PLC 源码。
- 参数校验、Motion Function 点生成和凸轮表加载。
- 轴上电、工艺坐标参考、凸轮耦合、连续运行、受控停止、脱开和从轴回程。
- 仿真/实机共用的轴接口。
- 在线监视和 Scope 专用状态结构。
- 离线数学校验、工程结构校验和运行时验收说明。
- 需要在 TwinCAT XAE 中人工执行的配置、编译、激活和 Scope 步骤。

### 3.2 不包含

- PLC Visualization 或 TwinCAT HMI。
- 物体传感器触发；物体只按主轴位置周期出现。
- 四条边的完整加工；只模拟两条边。
- 未知机械结构的实机回零程序、驱动整定和安全逻辑。
- 使用标准 PLC 逻辑替代急停、硬限位或 TwinSAFE。

## 4. 坐标和默认配方

### 4.1 坐标定义

凸轮表采用相对耦合坐标。每次开始运行时，当前传送带位置定义为主轴凸轮相位 0，从轴已确认的准备位置定义为从轴凸轮位置 0。

在加工区间内，用下式描述切割头在物体坐标系中的位置：

```text
r = SlaveCamPosition - MasterCamPosition
```

其中 `r` 只用于描述切割几何。Axis 2 仍执行凸轮表中的绝对从轴位移，并在每个周期末返回从轴凸轮位置 0。

### 4.2 默认配方

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `lrObjectPitch` | 500 mm | 相邻物体的主轴节距，也是凸轮周期 |
| `lrConveyorVelocity` | 100 mm/s | 传送带恒定速度 |
| `lrCutVelocity` | 50 mm/s | 切割头相对物体的速度绝对值 |
| `lrObjectEdgeLength` | 25 mm | 每条被模拟边的完整长度 |
| `lrFirstEdgeStart` | 100 mm | 第一边在主轴周期中的起点 |
| `lrSecondEdgeStart` | 300 mm | 第二边在主轴周期中的起点 |
| `lrBlendMasterDistance` | 20 mm | 切割速度与同步速度间的平滑过渡距离 |
| `lrReturnEndPhase` | 480 mm | 从轴完成回程的最晚主轴相位 |
| `lrMasterAcceleration` | 500 mm/s^2 | 主轴启动加速度默认值 |
| `lrMasterDeceleration` | 500 mm/s^2 | 主轴受控停止减速度默认值 |
| `lrMaxSlaveVelocity` | 600 mm/s | 配方允许的从轴速度绝对值上限 |
| `lrMaxSlaveAcceleration` | 1600 mm/s^2 | 配方允许的从轴加速度绝对值上限 |
| `lrMaxSlaveJerk` | 12000 mm/s^3 | 配方允许的从轴加加速度绝对值上限 |

单边切割时间和主轴切割距离由边长计算：

```text
CutTime = ObjectEdgeLength / CutVelocity
CutMasterDistance = CutTime * ConveyorVelocity
```

默认值对应：

```text
CutTime = 25 / 50 = 0.5 s
CutMasterDistance = 0.5 * 100 = 50 mm
```

禁止把“两条边起点的从轴偏移”作为独立工艺参数。两条边的位置必须由物体边长、主轴相位、切割速度和过渡段共同推导。

## 5. Motion Function 凸轮几何

### 5.1 速度斜率

凸轮表中 `SlaveVelo` 表示从轴位置相对于主轴位置的导数。默认斜率为：

```text
FirstCutSlope  = (ConveyorVelocity + CutVelocity) / ConveyorVelocity = 1.5
SyncSlope      = 1.0
SecondCutSlope = (ConveyorVelocity - CutVelocity) / ConveyorVelocity = 0.5
```

因此，当主轴以 100 mm/s 运行时：

- 第一边的从轴设定速度为 150 mm/s。
- 同速段的从轴设定速度为 100 mm/s。
- 第二边的从轴设定速度为 50 mm/s。

### 5.2 默认插值点

默认 Motion Function 使用以下几何点。位置均为相对耦合后的凸轮坐标。

| 主轴位置 X | 从轴位置 Y | 边界斜率 Y' | 后续区间用途 |
|---:|---:|---:|---|
| 0 | 0 | 0.0 | 周期起点 |
| 20 | 0 | 0.0 | 结束准备停留 |
| 100 | 100 | 1.5 | 到达第一边起点 |
| 150 | 175 | 1.5 | 完成第一边 |
| 170 | 200 | 1.0 | 平滑进入同步速度 |
| 280 | 310 | 1.0 | 结束同速跟随 |
| 300 | 325 | 0.5 | 到达第二边起点 |
| 350 | 350 | 0.5 | 完成第二边 |
| 480 | 0 | 0.0 | 完成线性轴快速回程 |
| 500 | 0 | 0.0 | 周期结束停留 |

切割几何为：

```text
第一边：(100, 100) -> (150, 175)
第一边物体相对位置：r = 0 -> 25 mm

第二边：(300, 325) -> (350, 350)
第二边物体相对位置：r = 25 -> 0 mm
```

第一边离开后的 20 mm 过渡会把物体相对位置从 25 mm 平滑移到 30 mm。`170..280 mm` 同速区间保持 `r = 30 mm`，第二边之前的 20 mm 过渡再把 `r` 从 30 mm 平滑移回 25 mm。因此，切割头在两次恒速切割之间不会停止，也不会把两条边误认为同一位置。

### 5.3 插值函数

- 停留和恒速区间使用 `MOTIONFUNCTYPE_POLYNOM1`。
- 追赶、切割/同步速度切换以及回程使用 `MOTIONFUNCTYPE_POLYNOM5_MM`。
- 凸轮表类型使用 `MC_TABLETYPE_MOTIONFUNCTION`。
- `MC_CamTableSelect.Periodic` 设置为 `TRUE`。
- 周期首尾满足 `Y(0) = Y(500) = 0`、`Y'(0) = Y'(500) = 0`，不得产生位置或速度跳变。

配方改变后，构建器重新计算所有派生相位、位置和斜率；不得只缩放默认点而忽略边界条件。

按上述默认点、主轴恒速 100 mm/s 和零端点加速度计算，回程段的从轴速度峰值约为 527.1 mm/s、加速度峰值约为 1345.4 mm/s^2、加加速度峰值约为 10623.6 mm/s^3。默认配方上限为这些理论值保留少量裕量，但实机 NC 轴限制可能更低；若实际限制不足，必须降低主轴速度、延长回程区间或增大物体节距。

## 6. 软件分层

### 6.1 全局轴与外部接口

- `GVL_Motion`：只保存传送带轴和切割轴的 `AXIS_REF`。
- `GVL_CamControl`：保存操作命令、当前配方、状态和 Scope 数据。

全局变量不得直接包含运动状态机实现。

### 6.2 数据类型

- `E_CamMachineState`：机器状态枚举。
- `E_CamProcessPhase`：准备、追赶、第一边、同步、第二边、回程等工艺阶段。
- `E_CamApplicationError`：应用层错误码。
- `ST_CamRecipe`：可配置配方和运动限制。
- `ST_CamCommands`：单周期命令接口。
- `ST_CamStatus`：面向在线诊断的状态。
- `ST_CamScope`：适合直接加入 Scope 的标量信号。

### 6.3 功能块

- `FB_CamRecipeValidator`：校验参数范围、区间顺序、派生切割距离、周期连续性和运动限制。
- `FB_CamProfileBuilder`：生成 `MC_MotionFunctionPoint` 数组和 `MC_CAM_REF`，不直接控制轴。
- `FB_ElectronicCam`：封装 `MC_CamTableSelect`、`MC_CamIn`、`MC_CamOut` 和凸轮状态。
- `FB_CamMachine`：顶层状态机，协调轴上电、工艺参考、凸轮加载、运行、停止、脱开和回程。
- `MAIN`：只调用 `FB_CamMachine` 并传递全局接口。

每个功能块只承担一个职责。轴功能块的 `Execute` 边沿、`Busy`、`Done`、`Error` 和 `ErrorID` 必须在所属封装层内处理，不能散落在 `MAIN` 中。

## 7. 命令与状态机

### 7.1 命令

- `bEnable`：允许两根轴上电。
- `bReference`：轴静止且凸轮脱开时，将当前传送带位置对齐为新周期相位 0，并确认从轴准备位置为 0。
- `bApplyRecipe`：校验参数、生成并加载 Motion Function。
- `bStart`：耦合凸轮并启动主轴恒速运动。
- `bStop`：执行受控停止。
- `bReset`：复位运动功能块和已锁存报警。

命令采用上升沿语义。状态机必须拒绝不适用于当前状态的命令，并记录明确的应用状态或错误码。

### 7.2 正常状态流

```text
Disabled
  -> Referencing
  -> BuildingTable
  -> Ready
  -> CammingIn
  -> Running
  -> StoppingMaster
  -> CammingOut
  -> ReturningSlave
  -> Ready
```

运行中不得重建或覆盖凸轮表。只有主轴停止、凸轮脱开且从轴不受凸轮控制时，`bApplyRecipe` 才能生效。

### 7.3 停止恢复

无论停止命令发生在哪个相位，都执行：

1. 受控停止主轴；从轴保持凸轮耦合并随主轴减速。
2. 主轴确认静止后执行 `MC_CamOut`。
3. 从轴执行 `MC_MoveAbsolute(0)` 返回准备位置。
4. 回程完成后进入 `Ready`。

不得在中途停止位置直接重新建立新周期相位 0，否则会导致从轴物理准备位置漂移。

### 7.4 工艺参考与机械回零

`bReference` 只对齐本示例的工艺坐标，不替代实机机械回零。实机必须先通过已验证的回零、限位和安全流程建立可靠坐标，再允许本示例捕获周期起点。仿真模式可以在两轴静止时将当前位置设为 0。

## 8. 参数和运动限制

至少校验：

- `ObjectPitch > 0`。
- `ConveyorVelocity > 0`。
- `0 < CutVelocity < ConveyorVelocity`，确保第二边默认仍沿正方向切割。
- `ObjectEdgeLength > 0`。
- 两条切割区间及其过渡段严格位于一个周期内且互不重叠。
- 第二边结束位置小于回程结束相位。
- 周期首尾位置、速度和加速度连续。
- 凸轮计算出的最大从轴速度、加速度和回程需求不超过配方限制及实际 NC 轴限制。

恒定主轴速度下，从轴运动特性按 `SlaveVelocity = Y' * MasterVelocity`、`SlaveAcceleration = Y'' * MasterVelocity^2`、`SlaveJerk = Y''' * MasterVelocity^3` 计算。主轴处于加减速时，还必须考虑主轴加速度通过凸轮斜率产生的附加从轴加速度。

若用户修改配方导致回程时间不足，配方校验失败，系统不得加载凸轮表或启动轴。

## 9. 错误处理与安全

以下情况进入 `Fault`：

- 配方或派生几何无效。
- 轴未上电、未完成必要参考或存在 NC 错误。
- 凸轮表加载、耦合、脱开或回程失败。
- 运行中请求重建凸轮表。
- 计算或读取到的运动特性超过限制。

状态中分别保存：

- 应用层错误枚举。
- 发生错误的机器状态。
- 原始 Beckhoff `ErrorID`。
- 错误来源，例如主轴、从轴、凸轮加载或凸轮耦合。

复位不得自动启动轴。错误清除后，操作者必须重新确认参考、配方和启动命令。

标准 PLC 程序不构成安全功能。实机的急停、硬限位、安全转矩关闭、许可和 TwinSAFE 由独立安全设计负责。

## 10. Scope 与在线诊断

`ST_CamScope` 至少提供：

- `lrMasterSetPosition`
- `lrMasterActualPosition`
- `lrMasterSetVelocity`
- `lrMasterActualVelocity`
- `lrMasterPhase`
- `lrSlaveSetPosition`
- `lrSlaveActualPosition`
- `lrSlaveSetVelocity`
- `lrSlaveActualVelocity`
- `lrExpectedSlaveVelocity`
- `lrObjectRelativePosition`
- `eMachineState`
- `eProcessPhase`
- `bCamActive`
- `bRecipeLoaded`
- `eApplicationError`
- `udiMotionErrorID`

推荐建立三组 Scope：

1. 主从轴设定速度、实际速度和理论从轴速度。
2. 主轴相位、从轴位置和物体相对位置。
3. 机器状态、工艺阶段、耦合状态和错误码。

## 11. 验收标准

### 11.1 仿真

- 主轴运行设定速度为 100 mm/s。
- `100..150 mm` 第一边区间，从轴设定速度为 150 mm/s，`r` 从 0 增加到 25 mm。
- `170..280 mm` 同速区间，从轴设定速度为 100 mm/s，`r` 保持 30 mm。
- `300..350 mm` 第二边区间，从轴设定速度为 50 mm/s，`r` 从 25 减少到 0 mm。
- 主轴相位到达 480 mm 前，从轴回到凸轮位置 0。
- 主轴相位从 500 mm 回绕到 0 时，从轴位置和速度连续。
- 中途停止后按顺序完成主轴停止、凸轮脱开和从轴回零。
- 停止后可以重新应用合法配方并再次启动。
- 运行中应用配方被拒绝，且当前凸轮表不被破坏。
- 全流程无未处理的运动功能块错误。

### 11.2 实机

- 先用降低后的速度、加速度和回程限制验证。
- 先验证设定曲线，再验证实际位置、速度和跟随误差。
- 实际误差阈值由驱动整定和机械能力确定，不在示例代码中假设统一固定值。
- 任何最大速度、加速度、限位或安全条件不满足时不得提高到默认速度。

## 12. 验证策略

- 为配方公式和默认插值点建立离线自动化测试，验证切割距离、关键点坐标、斜率和周期连续性。
- 为非法速度、边长过大、区间重叠和回程空间不足建立失败测试。
- 对 PLC 工程 XML 和引用关系做结构校验。
- 在 TwinCAT XAE 中执行 PLC 编译；如果本地命令行不可用，由用户按实施文档中的步骤完成并反馈编译信息。
- 在两根 NC 仿真轴上执行状态机与 Scope 验收。
- 仿真通过后再切换实轴映射并低速验证。

## 13. 人工配置边界

以下操作需要在 TwinCAT XAE 中完成或确认：

- TF5050 NC Camming 许可可用。
- `GVL_Motion` 的两个 `AXIS_REF` 分别链接到 NC Axis 1 和 Axis 2。
- 仿真时，将两根 NC 轴的编码器和驱动配置为仿真；实机时恢复 EtherCAT 映射。
- 配置实机缩放、软件限位、最大速度、加速度、减速度和机械回零。
- 激活配置、重启 TwinCAT Runtime、登录 PLC 并创建 Scope 工程。

PLC 源码不得通过一个运行时 BOOL 动态切换真实驱动和仿真驱动。

## 14. 官方依据

- [Beckhoff `MC_CamTableSelect`](https://infosys.beckhoff.com/content/1033/tf5050_tc3_nc_camming/1002328715.html)：从 PLC 的 `MC_CAM_REF` 创建并加载凸轮表。
- [Beckhoff `MC_CamIn`](https://infosys.beckhoff.com/content/1033/tf5050_tc3_nc_camming/1002438027.html)：建立主轴、从轴与指定凸轮表的耦合。
- [Beckhoff `MC_MotionFunctionPoint`](https://infosys.beckhoff.com/content/1033/tf5050_tc3_nc_camming/1003721995.html)：描述 Motion Function 的点、函数类型、位置、速度、加速度和加加速度边界。
- [Beckhoff `MC_MotionFunctionType`](https://infosys.beckhoff.com/content/1033/tf5050_tc3_nc_camming/1003744011.html)：`MOTIONFUNCTYPE_POLYNOM1` 用于同步直线，`MOTIONFUNCTYPE_POLYNOM5_MM` 用于运动到运动的五次多项式连接。
- [Beckhoff TF5050 库概览](https://infosys.beckhoff.com/content/1033/tf5050_tc3_nc_camming/6961172235.html)：`Tc2_MC2_Camming` 3.4.0 需要至少 TwinCAT Runtime 3.1.4024.55；当前工程版本满足该下限。
