# 实物电子凸轮操作与验证

## 适用配置

- 主轴：传送带，正方向运行，周期 `63.7 mm`
- 从轴：Cutter，可信的 NC 绝对位置，待机位置 `17 mm`
- 传送带生产及自动标定速度：`5 mm/s`
- 切割相对速度：`3 mm/s`
- 每条模拟边长度：`3 mm`
- 凸轮方式：相对凸轮，主轴与从轴均采用相对耦合

传送带上电后的原始 NC 坐标必须可信。程序使用 `MC_MoveAbsolute` 移动到原始 NC `-0.6 mm`，随后只对传送带执行 `MC_SetPosition(0.0)`。此流程不是传感器寻零，无法修正不可信的原始 NC 坐标。

Cutter 不执行位置重定义，只通过 `MC_MoveAbsolute` 移动到可信的绝对位置 `17 mm`。

## 每次上电后的启动顺序

1. 确认机械区域安全、两轴无故障且均已停止。
2. 将 `GVL_CamControl.stCommands.bEnable` 置为 `TRUE`，等待状态进入 `CAM_STATE_IDLE`。
3. 给 `bReference` 一个上升沿，程序自动依次执行：
   - `CAM_STATE_POSITIONING_MASTER`：以 `5 mm/s` 将传送带绝对定位到原始 NC `-0.6 mm`；
   - `CAM_STATE_REFERENCING_MASTER`：确认到位且停止后，将传送带当前位置定义为 `0 mm`；
   - `CAM_STATE_POSITIONING_SLAVE`：将 Cutter 绝对定位到 `17 mm`；
   - 完成后返回 `CAM_STATE_IDLE`，并令 `bReferenced=TRUE`。
4. 给 `bApplyRecipe` 一个上升沿，等待 `bRecipeLoaded=TRUE` 且状态进入 `CAM_STATE_READY`。
5. 确认传送带为 `0 mm`、Cutter 为 `17 mm` 且两轴停止，然后给 `bStart` 一个上升沿。
6. 程序先在静止状态耦合凸轮，耦合成功后才启动传送带；进入 `CAM_STATE_RUNNING` 后开始循环。

完整标定状态路径为：

```text
CAM_STATE_IDLE
-> CAM_STATE_POSITIONING_MASTER
-> CAM_STATE_REFERENCING_MASTER
-> CAM_STATE_POSITIONING_SLAVE
-> CAM_STATE_IDLE (bReferenced = TRUE)
```

`bReference`、`bApplyRecipe`、`bStart`、`bStop` 和 `bReset` 均按上升沿处理，操作后应恢复为 `FALSE`。

## 凸轮表

| 点 | 主轴 X (mm) | Cutter 相对 Y (mm) | 斜率 dY/dX |
|---:|---:|---:|---:|
| 1 | 0 | 0 | 0 |
| 2 | 1 | -1 | -0.4 |
| 3 | 6 | -3 | -0.4 |
| 4 | 10 | -7 | -1.6 |
| 5 | 15 | -15 | -1.6 |
| 6 | 17 | -16.6 | 0 |
| 7 | 27 | 0 | 0 |
| 8 | 63.7 | 0 | 0 |

以 Cutter 绝对 `17 mm` 为起点时，已确认的最小位置为 `0.4 mm`。各区间为：

- `0–1 mm`：切入准备
- `1–6 mm`：第一条边，Cutter 速度 `-(5-3)=-2 mm/s`，先向后切
- `6–10 mm`：五次多项式从 `-2 mm/s` 直接过渡到 `-8 mm/s`，不设置同步跟随平台
- `10–15 mm`：第二条边，Cutter 速度 `-(5+3)=-8 mm/s`，再向前切
- `15–17 mm`：退出并减速到零
- `17–27 mm`：两秒五次多项式从绝对 `0.4 mm` 回到绝对 `17 mm`，位移 `16.6 mm`
- `27–63.7 mm`：保持绝对 `17 mm`

两轴物理方向相反，因此用于观察切割效果的物体相对位置为：

```text
-(Cutter 相对凸轮位移 + 传送带主轴相位)
```

关键端点应依次显示 `0 → -3 → -3 → 0 mm`：第一条边从 `0` 到 `-3 mm`，平滑过渡段的两个端点保持 `-3 mm`，第二条边从 `-3 mm` 返回 `0`。

## 正常停止与再次启动

给 `bStop` 一个上升沿后，程序等待进入 `27–63.7 mm` 的零斜率驻留区，再受控停止传送带、解除凸轮耦合并把 Cutter 移回绝对 `17 mm`。

受减速距离影响，传送带通常不会恰好停在下一周期的 `0 mm`。如果停止结束后状态为 `CAM_STATE_IDLE` 且 `bReferenced=FALSE`，确认两轴停止后再次给 `bReference` 上升沿。程序会自动返回可信的原始 NC `-0.6 mm` 并重新置零，无需手动移动传送带；随后重新执行 `bApplyRecipe` 和 `bStart`。

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

首次实机测试时，应重点确认传送带能够安全移动到原始 NC `-0.6 mm`、Cutter 绝对位置不低于已确认的 `0.4 mm`、两个直线段实际速度依次接近 `-2 mm/s` 和 `-8 mm/s`，并确认回程没有超出轴的速度、加速度和加加速度限制。两秒回程的理论峰值约为速度 `15.5625 mm/s`、加速度 `23.96 mm/s²`、加加速度 `124.5 mm/s³`。

## 故障处理

发生故障时先记录 `eApplicationError`、`eErrorSource` 和 `udiMotionErrorID`。排除机械与 NC 故障并确认两轴停止后，给 `bReset` 一个上升沿。

传送带自动定位或置零失败会报告 `CAM_ERROR_AXIS_REFERENCE` 和 `CAM_SOURCE_MASTER_AXIS`。复位不会重新定义 Cutter 的绝对位置；再次启动前仍须确认传送带原始 NC 坐标可信，并检查 Cutter 的 `17 mm` 待机位置。
