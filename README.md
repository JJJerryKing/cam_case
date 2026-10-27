# TwinCAT 电子凸轮切割模拟（仿真）

这是一个基于 TwinCAT 3 的双轴电子凸轮项目。传送带作为主轴，Cutter 作为从轴，通过相对电子凸轮模拟传送带上物体两条边的连续切割过程。

项目提供以下运行流程：

```text
轴使能 → 位置标定 → 应用配方 → 凸轮耦合 → 连续运行 → 停止并回位
```

## 默认配置

| 项目 | 默认值 |
|---|---:|
| 主轴 | 传送带轴 `gAxisConveyor` |
| 从轴 | Cutter 轴 `gAxisCutter` |
| 两轴物理方向 | 相反 |
| 传送带周期 | `63.7 mm` |
| 传送带速度 | `5 mm/s` |
| 切割速度 | `3 mm/s` |
| 模拟边长 | `3 mm` |
| Cutter 待机位置 | `17 mm` |
| 位置允许误差 | `0.1 mm` |

## 工程环境

项目需要能够打开 TwinCAT 解决方案并配置 PLC 与 NC Motion 的 TwinCAT 3/XAE 环境。

当前工程文件记录的版本为：

- TwinCAT System Manager：`3.1.4024.75`
- PLC Project：`3.1.4024.0`

PLC 工程引用以下 Beckhoff 库：

- `Tc2_Math`
- `Tc2_Standard`
- `Tc2_System`
- `Tc3_Module`
- `Tc2_MC2`
- `Tc2_MC2_Camming`

打开工程后，请确认这些库均已正确解析，并准备两根已经配置的 NC 轴。

## 项目结构

```text
cam_case.sln
├─ cam_case/
│  ├─ cam_case.tsproj                 TwinCAT 系统工程
│  └─ Untitled1/                      PLC 工程
│     ├─ GVLs/GVL_Motion.TcGVL       两根轴的全局引用
│     ├─ GVLs/GVL_CamControl.TcGVL   操作命令、配方和状态入口
│     ├─ DUTs/                        命令、配方和状态数据类型
│     └─ POUs/                        主程序与电子凸轮程序
└─ docs/
   └─ twincat-physical-cam-operation.md
```

主要入口：

- [TwinCAT 解决方案](cam_case.sln)
- [PLC 工程](cam_case/Untitled1/Untitled1.plcproj)
- [控制变量](cam_case/Untitled1/GVLs/GVL_CamControl.TcGVL)
- [详细操作文档](docs/twincat-physical-cam-operation.md)

## 下载和打开工程

克隆仓库：

```powershell
git clone https://github.com/JJJerryKing/cam_case.git
cd cam_case
```

然后按以下顺序操作：

1. 使用 TwinCAT XAE 打开 `cam_case.sln`。
2. 选择需要使用的 TwinCAT Target System。
3. 确认 PLC 工程中的库引用均已解析。
4. 在 TwinCAT NC 配置中准备传送带轴和 Cutter 轴。
5. 将传送带轴对应到 `GVL_Motion.gAxisConveyor`。
6. 将 Cutter 轴对应到 `GVL_Motion.gAxisCutter`。
7. 生成解决方案并处理所有编译错误。
8. 激活 TwinCAT 配置，使系统进入运行模式。
9. 登录 PLC 并启动 PLC 程序。

## 启动顺序

操作入口为：

```text
GVL_CamControl.stCommands
```

除 `bEnable` 外，其余操作命令按上升沿处理。命令触发后应恢复为 `FALSE`，下一次操作前再重新置为 `TRUE`。

### 1. 使能轴

```iecst
GVL_CamControl.stCommands.bEnable := TRUE;
```

等待：

```text
GVL_CamControl.stStatus.bPowerEnabled = TRUE
GVL_CamControl.stStatus.eState = CAM_STATE_IDLE
```

### 2. 执行标定

给以下变量一个上升沿：

```iecst
GVL_CamControl.stCommands.bReference := TRUE;
```

随后恢复为：

```iecst
GVL_CamControl.stCommands.bReference := FALSE;
```

等待：

```text
GVL_CamControl.stStatus.bReferenced = TRUE
GVL_CamControl.stStatus.eState = CAM_STATE_IDLE
```

### 3. 应用配方

给以下变量一个上升沿：

```iecst
GVL_CamControl.stCommands.bApplyRecipe := TRUE;
```

随后恢复为 `FALSE`，并等待：

```text
GVL_CamControl.stStatus.bRecipeLoaded = TRUE
GVL_CamControl.stStatus.eState = CAM_STATE_READY
```

### 4. 启动运行

给以下变量一个上升沿：

```iecst
GVL_CamControl.stCommands.bStart := TRUE;
```

随后恢复为 `FALSE`，并等待：

```text
GVL_CamControl.stStatus.bCamActive = TRUE
GVL_CamControl.stStatus.bRunning = TRUE
GVL_CamControl.stStatus.eState = CAM_STATE_RUNNING
```

### 5. 停止运行

给以下变量一个上升沿：

```iecst
GVL_CamControl.stCommands.bStop := TRUE;
```

随后恢复为 `FALSE`。停止流程完成后，状态将回到 `CAM_STATE_READY` 或 `CAM_STATE_IDLE`。

如果回到 `CAM_STATE_IDLE` 且 `bReferenced=FALSE`，再次运行前依次重新执行：

```text
bReference → bApplyRecipe → bStart
```

## 操作变量速查

| 变量 | 使用方式 | 作用 |
|---|---|---|
| `bEnable` | 持续保持 | 使能或关闭设备运行请求 |
| `bReference` | 上升沿 | 执行位置标定 |
| `bApplyRecipe` | 上升沿 | 应用当前配方 |
| `bStart` | 上升沿 | 耦合电子凸轮并启动运行 |
| `bStop` | 上升沿 | 请求停止运行 |

## 关键状态速查

| 状态或变量 | 含义 |
|---|---|
| `CAM_STATE_IDLE` | 设备已使能，等待标定或应用配方 |
| `CAM_STATE_READY` | 标定与配方已准备完成，等待启动 |
| `CAM_STATE_RUNNING` | 电子凸轮正在运行 |
| `bPowerEnabled` | 两根轴均已使能 |
| `bReferenced` | 当前运行所需的位置标定已完成 |
| `bRecipeLoaded` | 当前配方已经应用 |
| `bCamActive` | 主从轴电子凸轮已经耦合 |
| `bRunning` | 设备处于连续运行状态 |
| `bBusy` | 自动流程正在执行 |

更多操作信息请参阅[详细操作文档](docs/twincat-physical-cam-operation.md)。
