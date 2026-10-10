# 运行配对与失败模块证据

运行：unified_r43_20261010_devreg130；对照：route_seam_20261008_devreg130。

```json
{
  "run": "unified_r43_20261010_devreg130",
  "parent": "route_seam_20261008_devreg130",
  "scenes": 130,
  "set_scope": "original_devreg130",
  "comparison_scope": "paired_driving",
  "outcomes": {
    "到达": 115,
    "碰撞": 3,
    "超时": 12
  },
  "source_identity_passed": true,
  "runtime_identity_passed": true,
  "state_identical_count": 0,
  "action_identical_count": 0,
  "lost_successes": [
    "highway_merge_2_1_141",
    "highway_merge_2_2_265",
    "1346cutin71",
    "7_916_merge_921"
  ],
  "new_collisions": [
    "7_916_merge_921"
  ],
  "errors": [],
  "task_input_invalid": [],
  "algorithm_errors": [],
  "outcome_classes": {
    "到达": 115,
    "碰撞": 3,
    "超时": 12
  },
  "input_protocol_mismatches": [],
  "gained_successes": [
    "12_36_straight_in_adjacent_left_37",
    "12_60_straight_in_adjacent_left_64",
    "13_72_straight_straight_75",
    "14_14_left_straight_in_adjacent_38",
    "18_34_straight_in_opposite_left_37",
    "20_94_straight_straight_97",
    "8_282_merge_291",
    "highway_merge_2_1_112",
    "highway_merge_3_1_86",
    "highway_merge_3_2_127",
    "19_67_straight_straight_70",
    "highway_merge_2_2_362"
  ],
  "execution_state_mismatches": 0,
  "mapped_audit_identity": {
    "path": "E:\\Onsite比赛\\onsite_challenge\\research\\arb_capability_20261007\\unified_execution_20261009\\runs\\unified_r43_20261010_devreg130\\mapped_body_audit\\summary.json",
    "sha256": "a22d914ce67118f3c474934b44ab1833eb396be7fb81ebba355e665163ac4014",
    "tool_sha256": "e4c82928c3dd87125cf257565791aa70fb1137470f5aaacf7dbb47c8afb90bd9",
    "geometry_tool_sha256": "4f585f85d470870d1a2c43010fdec33cd37f06f2f2468b5793ea417dedd4fd88"
  },
  "mapped_body_checked_count": 130,
  "mapped_body_exterior_growth": [
    "12_60_straight_in_adjacent_left_64",
    "14_123_straight_in_opposite_left_124",
    "8_282_merge_291"
  ],
  "mapped_body_initial_exterior_unrecovered": [],
  "mapped_body_grid_passed": false,
  "mapped_body_status": "issues_observed",
  "arrived_with_mapped_exterior_growth": [
    "12_60_straight_in_adjacent_left_64",
    "14_123_straight_in_opposite_left_124",
    "8_282_merge_291"
  ],
  "latency_pass_count": 72,
  "promotion_allowed": false,
  "driving_pair_passed": false,
  "driving_pair_scope": "Official recorded outcomes and execution/input parity; actual mapped-body acceptance is separate",
  "review_tool_sha256": "2cf8450c9c658686db423bd07db026d3b9f349751ab413f10b1a34d93e666450",
  "limitations": [
    "Module labels are evidence or coupled hypotheses, not automatic causal proofs.",
    "Mapped boundary diagnostic is not a legal-road or continuous-sweep certificate."
  ]
}
```

模块归因是证据分类；耦合问题需要逐因素反事实诊断，不把候选无解等同于物理无解。

| 场景 | 原结果→本次 | 无可行候选帧 | 未来车身出地图帧 | 实际外部面积首次增长 | 模块证据 |
|---|---|---:|---:|---:|---|
| 12_60_straight_in_adjacent_left_64 | 碰撞→到达 | 5 | 144 | 20.5 | candidate_generation + occupancy_prediction + execution_constraints; route + controller + mapped_boundary; route + controller + mapped_boundary; route + controller + mapped_boundary + risk_selection |
| 14_123_straight_in_opposite_left_124 | 到达→到达 | 123 | 142 | 0.1 | candidate_generation + occupancy_prediction + execution_constraints; route + controller + mapped_boundary; route + controller + mapped_boundary; route + controller + mapped_boundary + risk_selection |
| 15_31_straight_straight_37 | 碰撞→碰撞 | 0 | 32 | None | route + controller + mapped_boundary; visibility_birth |
| 18_108_straight_straight_112 | 碰撞→碰撞 | 72 | 0 | None | candidate_generation + occupancy_prediction + execution_constraints; prediction + occupancy + candidate_selection |
| 8_282_merge_291 | 碰撞→到达 | 206 | 205 | 27.8 | candidate_generation + occupancy_prediction + execution_constraints; route + controller + mapped_boundary; route + controller + mapped_boundary; route + controller + mapped_boundary + risk_selection |
| 9_381_merge_397 | 超时→超时 | 0 | 0 | None | route + candidate_progress + selection_budget |
| highway_merge_2_1_141 | 到达→超时 | 0 | 0 | None | route + candidate_progress + selection_budget |
| highway_merge_2_2_0 | 超时→超时 | 0 | 1 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| highway_merge_2_2_265 | 到达→超时 | 0 | 0 | None | route + candidate_progress + selection_budget |
| highway_merge_3_1_197 | 超时→超时 | 0 | 11 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| highway_merge_3_1_332 | 超时→超时 | 0 | 12 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| highway_merge_3_1_391 | 超时→超时 | 0 | 10 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| highway_merge_3_1_87 | 超时→超时 | 0 | 10 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| 1346cutin71 | 到达→超时 | 0 | 0 | None | route + candidate_progress + selection_budget |
| 7_916_merge_921 | 到达→碰撞 | 33 | 11 | None | candidate_generation + occupancy_prediction + execution_constraints; route + controller + mapped_boundary; prediction + occupancy + candidate_selection |
| highway_merge_2_1_464 | 超时→超时 | 0 | 86 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| highway_merge_3_1_301 | 超时→超时 | 0 | 84 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| highway_merge_3_1_68 | 超时→超时 | 0 | 10 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |

逐候选淘汰原因、成本分项、首冲突对象/分支、执行序列及下一帧对账在每场原始压缩日志中。
首次/末次风险帧与全部失败分类见 `failure_attribution.json`。

官方回放结局保留；到达但实际车身外部面积增长的场景也进入问题表。
初始外伸单独记录，地图覆盖核查不等同于合法道路或连续扫掠认证。

## 决策链首次分歧

| 场景 | 首次动作分歧 | 占用/跟踪见证丢失 | 道路见证丢失 | 有候选但预算丢失 | 最后前缀可行决策 |
|---|---:|---:|---:|---:|---:|
| 12_60_straight_in_adjacent_left_64 | 0.0 | 20.3 | 20.5 |  | 29.4 |
| 14_123_straight_in_opposite_left_124 | 0.0 | 3.8 | 0.0 |  | 26.2 |
| 15_31_straight_straight_37 | 0.0 |  |  |  | 6.6 |
| 18_108_straight_straight_112 | 0.1 | 1.3 |  |  | 4.6 |
| 8_282_merge_291 | 0.0 | 4.5 | 4.2 |  | 42.3 |
| 9_381_merge_397 | 0.8 |  |  | 60.3 | 67.9 |
| highway_merge_2_1_141 | 0.0 |  |  | 3.8 | 9.9 |
| highway_merge_2_2_0 | 0.0 |  |  | 5.3 | 9.9 |
| highway_merge_2_2_265 | 0.0 |  |  | 3.0 | 9.9 |
| highway_merge_3_1_197 | 0.0 |  |  | 0.5 | 9.9 |
| highway_merge_3_1_332 | 0.0 |  |  | 1.4 | 9.9 |
| highway_merge_3_1_391 | 0.0 |  |  | 1.5 | 9.9 |
| highway_merge_3_1_87 | 0.0 |  |  | 1.5 | 9.9 |
| 1346cutin71 | 0.0 |  |  | 3.96 | 9.96 |
| 7_916_merge_921 | 0.2 | 31.8 |  |  | 32.6 |
| highway_merge_2_1_464 | 0.0 |  |  | 0.5 | 9.9 |
| highway_merge_3_1_301 | 0.3 |  |  | 0.3 | 9.9 |
| highway_merge_3_1_68 | 0.0 |  |  | 1.5 | 9.9 |

表中候选见证来自当帧已保存的评估，不是反事实闭环结果；道路未查、预测支和成本分项均在JSON中保留。
