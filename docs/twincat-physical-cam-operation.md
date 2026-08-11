# 实物电子凸轮操作与验证

## 适用配置

- 主轴：传送带，正方向运行，周期 `63.7 mm`
- 从轴：Cutter，可信的 NC 绝对位置，待机位置 `17 mm`
- 传送带速度：`5 mm/s`
- 切割相对速度：`3 mm/s`
- 每条模拟边长度：`3 mm`
- 凸轮方式：相对凸轮，主轴与从轴均使用相对耦合

程序只对传送带调用 `MC_SetPosition(0.0)`。Cutter 不执行位置重定义，而是通过绝对定位移动到 `17 mm`。

## 每次上电后的启动顺序

1. 确认机械安全、两轴无故障，并让两轴停止。
2. 手动移动传送带，使物理基准标记对准；此时传送带 NC 实际位置应约为 `-0.6 mm`，允许误差由 `lrPositionTolerance` 决定，默认 `0.1 mm`。
3. 将 `GVL_CamControl.stCommands.bEnable` 置为 `TRUE`，等待状态进入 `CAM_STATE_IDLE`。
4. 给 `bReference` 一个上升沿。程序将：
   - 在当前位置把传送带 NC 位置设为 `0 mm`；
   - 把 Cutter 绝对定位到 `17 mm`；
   - 完成后令 `bReferenced=TRUE`。
5. 给 `bApplyRecipe` 一个上升沿，等待 `bRecipeLoaded=TRUE` 且状态进入 `CAM_STATE_READY`。
6. 再次确认传送带为 `0 mm`、Cutter 为 `17 mm` 且两轴停止，然后给 `bStart` 一个上升沿。
7. 程序先在静止状态耦合凸轮，耦合成功后才启动传送带；进入 `CAM_STATE_RUNNING` 后开始循环。

`bReference`、`bApplyRecipe`、`bStart`、`bStop` 和 `bReset` 都按上升沿处理，操作后应恢复为 `FALSE`。

## 凸轮表

| 点 | 主轴 X (mm) | Cutter 相对 Y (mm) | 斜率 dY/dX |
|---:|---:|---:|---:|
| 1 | 0 | 0 | 0 |
| 2 | 1 | -1 | -1.6 |
| 3 | 6 | -9 | -1.6 |
| 4 | 11 | -14 | -0.4 |
| 5 | 16 | -16 | -0.4 |
| 6 | 17 | -16.2 | 0 |
| 7 | 27 | 0 | 0 |
| 8 | 63.7 | 0 | 0 |

以 Cutter 绝对 `17 mm` 为起点时，最小位置约为 `0.8 mm`。各区间为：

- `0–1 mm`：切入准备
- `1–6 mm`：第一条边，Cutter 速度 `-(5+3)=-8 mm/s`
- `6–11 mm`：五次多项式直接过渡，不设置同步跟随平台
- `11–16 mm`：第二条边，Cutter 速度 `-(5-3)=-2 mm/s`
- `16–17 mm`：退出并减速到零
- `17–27 mm`：两秒五次多项式回到绝对 `17 mm`
- `27–63.7 mm`：保持绝对 `17 mm`

两轴物理方向相反，因此用于观察切割效果的物体相对位置为：

```text
Cutter 相对凸轮位移 + 传送带主轴相位
```

两段直线切割期间，该值分别变化 `-3 mm` 和 `+3 mm`。

## 正常停止与再次启动

给 `bStop` 一个上升沿后，程序等待进入 `27–63.7 mm` 的零斜率驻留区，再按受控减速度停止、解耦并把 Cutter 移回绝对 `17 mm`。

受减速距离影响，传送带通常不会恰好停在下一周期的 `0 mm`。如果停止结束后状态为 `CAM_STATE_IDLE` 且 `bReferenced=FALSE`，再次运行前必须重新将物理基准移到 NC 约 `-0.6 mm`，然后重复 `bReference`、`bApplyRecipe` 和 `bStart`。不要在任意停点直接置零，否则凸轮周期会与传送带上的物体标记错位。

## Scope 建议

至少同时观察：

- `GVL_CamControl.stScope.lrMasterPhase`
- `lrMasterSetPosition`、`lrMasterActualPosition`
- `lrSlaveSetPosition`、`lrSlaveActualPosition`
- `lrSlaveSetVelocity`、`lrSlaveActualVelocity`
- `lrExpectedSlaveVelocity`
- `lrObjectRelativePosition`
- `eMachineState`、`eProcessPhase`
- `bCamActive`、`eApplicationError`、`udiMotionErrorID`

首次低速实机测试时，应重点确认 Cutter 绝对位置始终高于机械下限、两个直线段的实际速度接近 `-8 mm/s` 和 `-2 mm/s`，以及回程区没有超出轴的速度、加速度和加加速度限制。

## 故障处理

发生故障时先记录 `eApplicationError`、`eErrorSource` 和 `udiMotionErrorID`。排除机械与 NC 故障并确认两轴停止后，给 `bReset` 一个上升沿。复位不会重新定义 Cutter 的绝对位置；重新启动前仍须检查传送带物理基准和 Cutter 的 `17 mm` 待机位置。
