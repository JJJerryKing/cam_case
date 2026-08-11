# 实物双边切割电子凸轮设计

## 1. 目标

将现有按500 mm周期运行的双边切割仿真改造成与实物标定坐标一致的周期凸轮：传送带每圈63.7 mm，Cutter以17 mm为待机位置，在一个周期内模拟四边形物体的两条3 mm边切割，随后快速回到17 mm并等待下一周期。

## 2. 已确认的实物坐标与工艺参数

- 传送带正向运行时，NC位置从0 mm递增。
- 传送带一整圈对应63.7 mm，亦为凸轮周期。
- 每次上电后，操作员将传送带置于其NC读数约为-0.6 mm的物理标定点，再用`MC_SetPosition(Position := 0.0, Mode := FALSE)`建立周期零点。
- Cutter绝对NC位置断电重启后可信，禁止在工艺参考流程中重新把Cutter当前位置设为0。
- Cutter耦合前待机位置为17 mm。
- 传送带速度为5 mm/s。
- 相对切割速度为3 mm/s。
- 每条模拟边长为3 mm。
- 第一边Cutter速度幅值为8 mm/s，第二边Cutter速度幅值为2 mm/s。
- Cutter切割方向与传送带正方向相反，因此两段Cutter速度及凸轮斜率均为负。

## 3. 相对凸轮坐标

保留现有`MC_STARTMODE_RELATIVE`。耦合瞬间：

- 传送带位于周期相位0；
- Cutter位于绝对位置17 mm；
- 凸轮主轴相对位置X为0；
- 凸轮从轴相对位置Y为0。

凸轮表只描述Cutter相对于17 mm耦合位置的位移。实际Cutter位置为：

```text
CutterAbsolute = 17.0 + CamY
```

两条边的斜率为：

```text
FirstSlope  = -(5 + 3) / 5 = -1.6
SecondSlope = -(5 - 3) / 5 = -0.4
```

每条边切割时间和对应主轴距离为：

```text
CutTime = 3 / 3 = 1 s
CutMasterDistance = 1 * 5 = 5 mm
```

## 4. 八点Motion Function凸轮表

| 点 | Master X (mm) | Cam Y (mm) | Cutter绝对位置 (mm) | Y' | 到下一点的函数 |
|---:|---:|---:|---:|---:|---|
| 1 | 0.0 | 0.0 | 17.0 | 0.0 | `MOTIONFUNCTYPE_POLYNOM5_MM` |
| 2 | 1.0 | -1.0 | 16.0 | -1.6 | `MOTIONFUNCTYPE_POLYNOM1` |
| 3 | 6.0 | -9.0 | 8.0 | -1.6 | `MOTIONFUNCTYPE_POLYNOM5_MM` |
| 4 | 11.0 | -14.0 | 3.0 | -0.4 | `MOTIONFUNCTYPE_POLYNOM1` |
| 5 | 16.0 | -16.0 | 1.0 | -0.4 | `MOTIONFUNCTYPE_POLYNOM5_MM` |
| 6 | 17.0 | -16.2 | 0.8 | 0.0 | `MOTIONFUNCTYPE_POLYNOM5_MM` |
| 7 | 27.0 | 0.0 | 17.0 | 0.0 | `MOTIONFUNCTYPE_POLYNOM1` |
| 8 | 63.7 | 0.0 | 17.0 | 0.0 | `MOTIONFUNCTYPE_NOTDEF` |

各区段含义：

- `0..1`：从静止平滑进入第一边速度，同时Cutter从17 mm到16 mm。
- `1..6`：第一边恒定切割，Cutter速度为-8 mm/s。
- `6..11`：用单个五次多项式从斜率-1.6直接过渡到-0.4，不设置同步跟随平台。
- `11..16`：第二边恒定切割，Cutter速度为-2 mm/s。
- `16..17`：用1 mm主轴安全区把Cutter平滑减速至0；Cutter最低位置为0.8 mm。
- `17..27`：用五次多项式在2 s内从0.8 mm平滑返回17 mm。
- `27..63.7`：Cutter保持17 mm，提供观察、停机和下一周期准备区。

周期首尾均满足Y=0、Y'=0，周期切换不产生位置或速度跳变。

## 5. 物体相对位置

旧仿真假定主从轴同向，使用：

```text
ObjectRelativePosition = SlaveCamPosition - MasterPhase
```

实物中Cutter切割方向与传送带正方向相反，应改为：

```text
ObjectRelativePosition = SlaveCamPosition + MasterPhase
```

其中`SlaveCamPosition`为Cutter当前设定位置减去耦合时Cutter位置。预期：

- 第一边：相对位置从0降到-3 mm；
- 中间平滑段：端点均为-3 mm；
- 第二边：相对位置从-3 mm回到0。

## 6. 上电标定与启动顺序

工艺参考流程只重定义传送带坐标，不修改Cutter绝对坐标：

1. 两轴上电并确认静止。
2. 操作员将传送带置于NC读数约-0.6 mm的物理标定位置。
3. 仅对传送带执行`MC_SetPosition(0.0)`；超出位置容差时拒绝参考命令。
4. 用`MC_MoveAbsolute(17.0)`将Cutter移动到待机位置。
5. 校验Cutter位于17 mm容差范围内。
6. 构建并加载凸轮表。
7. 两轴静止且传送带相位为0时执行相对`MC_CamIn`。
8. 凸轮耦合完成后，才用`MC_MoveVelocity`启动传送带至5 mm/s。

必须把现有“先启动主轴、达到恒速后再耦合”改为“先耦合、后启动”，否则凸轮相位0与传送带物理标记不一致。主轴从静止加速时从轴已经处于凸轮控制下，因此动态校验必须包含主轴加速对从轴的附加速度、加速度和加加速度影响。

## 7. 停止与恢复

- 正常停止优先在`27..63.7`零斜率待机区启动主轴受控停止。
- 主轴停止后执行`MC_CamOut`。
- 脱离凸轮后，Cutter用`MC_MoveAbsolute(17.0)`恢复待机位置。
- 故障停止继续使用受控停止流程，不得在运动中用`MC_SetPosition`修正坐标。
- 重新启动前重新确认传送带参考状态、Cutter 17 mm待机位置和有效凸轮表。

## 8. 配方与派生几何

配方默认值改为：

```text
ObjectPitch = 63.7 mm
ConveyorVelocity = 5.0 mm/s
CutVelocity = 3.0 mm/s
ObjectEdgeLength = 3.0 mm
FirstEdgeStart = 1.0 mm
InterEdgeTransitionDistance = 5.0 mm
ExitTransitionDistance = 1.0 mm
ReturnStartPhase = 17.0 mm
ReturnEndPhase = 27.0 mm
CutterReadyPosition = 17.0 mm
MasterReferenceRawPosition = -0.6 mm
```

派生几何负责计算两条负斜率、每边5 mm主轴距离、关键Y位置和回程位移。旧的“退出第一边—同步平台—进入第二边”派生字段和校验应合并为一个边间过渡区，不再要求同步平台长度大于0。

## 9. 状态、诊断与Scope

工艺阶段简化为：

```text
Prepare -> FirstEdge -> InterEdgeTransition -> SecondEdge
-> Exit -> Return -> Dwell
```

Scope至少验证：

- Master Phase按63.7 mm回绕；
- Cutter设定/实际位置范围约0.8..17 mm；
- 第一边设定速度约-8 mm/s；
- 第二边设定速度约-2 mm/s；
- 回程时间约2 s；
- 第一边相对位移0到-3 mm，第二边-3到0；
- 周期首尾位置、速度连续；
- 正常停止只在零斜率待机区触发。

## 10. 动态约束

17..27 mm回程对应2 s，位移16.2 mm。五次静止到静止曲线的估算峰值为：

```text
PeakReturnVelocity     ~= 15.2 mm/s
PeakReturnAcceleration ~= 23.4 mm/s^2
```

加载凸轮表后继续使用`MC_ReadCamTableCharacteristics`读取实际速度、加速度和加加速度特性，并同时与配方限制、NC轴限制和实机允许值比较。实机首次运行应降低主轴加速度和急动度，先观察设定曲线，再逐步允许实际运动。

## 11. 验收标准

- 传送带在-0.6 mm标定点执行参考后显示0 mm。
- 参考流程不调用Cutter的`MC_SetPosition`。
- 耦合前Cutter位于17 mm，且主轴位于周期0。
- 凸轮先耦合，传送带后启动。
- 第一边Master 1..6 mm、Cutter 16..8 mm、速度约-8 mm/s。
- 中间Master 6..11 mm使用连续五次曲线，无同步平台。
- 第二边Master 11..16 mm、Cutter 3..1 mm、速度约-2 mm/s。
- Master 16..17 mm内Cutter平滑减速，最低位置不低于0.8 mm设计值。
- Master 17..27 mm内Cutter在约2 s内返回17 mm。
- Master 27..63.7 mm内Cutter保持17 mm。
- 周期边界无位置、速度或加速度的不连续跳变。
- 正常停止、脱离凸轮和返回待机位置流程成功。

## 12. 官方依据

- [Beckhoff MC_StartMode](https://infosys.beckhoff.com/content/1033/tf5050_tc3_nc_camming/1003815691.html)：相对模式把耦合位置作为凸轮坐标原点。
- [Beckhoff MC_CamIn](https://infosys.beckhoff.com/content/1033/tf5050_tc3_nc_camming/1002438027.html)：建立主从轴凸轮耦合并解释相对/绝对起始模式。
- [Beckhoff MC_CamIn附录](https://infosys.beckhoff.com/content/1033/tf5050_tc3_nc_camming/1002498443.html)：耦合前从轴必须位于凸轮所要求的位置，避免设定位置跳变。

