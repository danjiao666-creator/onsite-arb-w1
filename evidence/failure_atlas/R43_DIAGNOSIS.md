# 纯 R43 失败诊断

本报告只看冻结纯 R43，不看目标关联后的 201 版，也不看联合 MPC。230 场原始结果已实核为 199 到达、8 碰撞、23 超时；回放另列 3 个“到达但实际车身道路外伸增长”事件，不修改原结果分母。

最容易失败的是高速合流：17/55 场，30.9%，全部超时。其次是切入 3/18（16.7%）和其余合流 4/25（16.0%）。从碰撞比例看，其余合流为 3/25（12.0%），交叉路口为 5/76（6.6%）。这些是当前 230 场固定样本的分布，不代表所有真实道路场景。

四层不能把问题统一归到预测：8 次碰撞中，2 次碰撞对象在终帧才首次可见；另外 6 场有“已预测到首步冲突，但无合格候选后仍返回风险缓解动作”的历史证据（其中一场涉及两个对象，共 7 条对象证据）。23 个超时均记录当时候选没有预计期限内到达方案；这不是物理无解证明。

20 个终局失败曾出现选中模型未来外伸的诊断，但实际发生道路外伸增长只有独立道路审计认定的 4 场：3 场原结果到达，另 1 场 `18_29_straight_straight_30` 超时。模型未来诊断和真实执行违规必须分开看。

Pure R43 only: 230 scenes, 199 arrivals, 8 collisions, 23 timeouts. Three arrivals with mapped-body exterior growth are shown separately and do not change these outcome counts.

## Failure Rate By Family

| Family | Failures / Total | Collisions | Timeouts | Rate |
|---|---:|---:|---:|---:|
| highway_merge | 17/55 | 0 | 17 | 30.9% |
| cutin | 3/18 | 0 | 3 | 16.7% |
| merge | 4/25 | 3 | 1 | 16.0% |
| intersection | 6/76 | 5 | 1 | 7.9% |
| lanechange | 1/14 | 0 | 1 | 7.1% |
| follow | 0/36 | 0 | 0 | 0.0% |
| other | 0/3 | 0 | 0 | 0.0% |
| rush | 0/3 | 0 | 0 | 0.0% |

## Four Layers

Prediction: saved one-step feedback and historical actor-birth evidence are available; future branch geometry was not saved. Small one-step errors do not certify long-horizon prediction.

Decision: per-frame selected ID, costs, ETA, prefix gap, feasible counts and risk status are preserved. No feasible generated candidate is not proof of physical impossibility.

Planning: candidate counts, occupancy/tracking/road rejections and original target/road geometry are visible. Full planned reference route points were not saved and are not reconstructed.

Control: actual action and exact next-state execution feedback are available. Selected model rollout is distinct from actual future motion and does not certify a good upstream decision.

## Events

- 12_60_straight_in_adjacent_left_64: arrival_road_violation; t=29.5s, remaining goal distance=0.713m.
- 14_123_straight_in_opposite_left_124: arrival_road_violation; t=26.3s, remaining goal distance=1.236m.
- 15_31_straight_straight_37: collision; t=6.7s, remaining goal distance=102.739m.
- 18_108_straight_straight_112: collision; t=11.3s, remaining goal distance=92.113m.
- 8_282_merge_291: arrival_road_violation; t=42.4s, remaining goal distance=2.177m.
- 9_381_merge_397: timeout; t=68.0s, remaining goal distance=66.973m.
- highway_merge_2_1_141: timeout; t=10.0s, remaining goal distance=89.643m.
- highway_merge_2_2_0: timeout; t=10.0s, remaining goal distance=92.813m.
- highway_merge_2_2_265: timeout; t=10.0s, remaining goal distance=30.159m.
- highway_merge_3_1_197: timeout; t=10.0s, remaining goal distance=68.732m.
- highway_merge_3_1_332: timeout; t=10.0s, remaining goal distance=100.524m.
- highway_merge_3_1_391: timeout; t=10.0s, remaining goal distance=71.351m.
- highway_merge_3_1_87: timeout; t=10.0s, remaining goal distance=81.843m.
- 1346cutin71: timeout; t=10.0s, remaining goal distance=49.278m.
- 7_916_merge_921: collision; t=35.8s, remaining goal distance=65.007m.
- highway_merge_2_1_464: timeout; t=10.0s, remaining goal distance=249.669m.
- highway_merge_3_1_301: timeout; t=10.0s, remaining goal distance=254.390m.
- highway_merge_3_1_68: timeout; t=10.0s, remaining goal distance=65.142m.
- 2422cutin168: timeout; t=8.84s, remaining goal distance=53.782m.
- 1lanechange42: timeout; t=14.8s, remaining goal distance=27.812m.
- highway_merge_3_1_223: timeout; t=10.0s, remaining goal distance=182.103m.
- 1cutin74: timeout; t=14.8s, remaining goal distance=49.014m.
- highway_merge_3_1_432: timeout; t=10.0s, remaining goal distance=249.481m.
- highway_merge_3_1_495: timeout; t=10.0s, remaining goal distance=105.925m.
- highway_merge_3_1_18: timeout; t=10.0s, remaining goal distance=74.376m.
- 7_661_merge_664: collision; t=34.2s, remaining goal distance=66.648m.
- highway_merge_2_1_482: timeout; t=10.0s, remaining goal distance=245.968m.
- highway_merge_3_1_26: timeout; t=10.0s, remaining goal distance=27.797m.
- highway_merge_2_2_118: timeout; t=10.0s, remaining goal distance=76.464m.
- 7_277_merge_292: collision; t=41.9s, remaining goal distance=66.723m.
- 13_91_straight_in_opposite_left_96: collision; t=11.2s, remaining goal distance=22.382m.
- 18_29_straight_straight_30: timeout; t=19.8s, remaining goal distance=92.584m.
- 11_57_straight_straight_61: collision; t=9.9s, remaining goal distance=104.190m.
- 20_46_straight_in_adjacent_left_47: collision; t=11.3s, remaining goal distance=63.925m.
