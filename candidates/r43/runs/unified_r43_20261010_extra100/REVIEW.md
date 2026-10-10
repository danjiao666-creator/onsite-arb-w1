# 运行配对与失败模块证据

运行：unified_r43_20261010_extra100；对照：unified_r26_20261010_extra100_normalized_baseline。

```json
{
  "run": "unified_r43_20261010_extra100",
  "parent": "unified_r26_20261010_extra100_normalized_baseline",
  "scenes": 100,
  "set_scope": "additional_development_not_holdout",
  "comparison_scope": "paired_driving",
  "outcomes": {
    "到达": 84,
    "超时": 11,
    "碰撞": 5
  },
  "source_identity_passed": true,
  "runtime_identity_passed": true,
  "state_identical_count": 100,
  "action_identical_count": 100,
  "lost_successes": [],
  "new_collisions": [],
  "errors": [],
  "task_input_invalid": [],
  "algorithm_errors": [],
  "outcome_classes": {
    "到达": 84,
    "超时": 11,
    "碰撞": 5
  },
  "input_protocol_mismatches": [],
  "gained_successes": [],
  "execution_state_mismatches": 0,
  "mapped_audit_identity": {
    "path": "E:\\Onsite比赛\\onsite_challenge\\research\\arb_capability_20261007\\unified_execution_20261009\\runs\\unified_r43_20261010_extra100\\mapped_body_audit\\summary.json",
    "sha256": "0add5616ca276333236787e9d3bc5525907802e588539ebf35b2b643cd3de864",
    "tool_sha256": "e4c82928c3dd87125cf257565791aa70fb1137470f5aaacf7dbb47c8afb90bd9",
    "geometry_tool_sha256": "4f585f85d470870d1a2c43010fdec33cd37f06f2f2468b5793ea417dedd4fd88"
  },
  "mapped_body_checked_count": 100,
  "mapped_body_exterior_growth": [
    "18_29_straight_straight_30"
  ],
  "mapped_body_initial_exterior_unrecovered": [],
  "mapped_body_grid_passed": false,
  "mapped_body_status": "issues_observed",
  "arrived_with_mapped_exterior_growth": [],
  "latency_pass_count": 56,
  "promotion_allowed": false,
  "driving_pair_passed": true,
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
| 2422cutin168 | 超时→超时 | 0 | 0 | None | route + candidate_progress + selection_budget |
| 1lanechange42 | 超时→超时 | 0 | 0 | None | route + candidate_progress + selection_budget |
| highway_merge_3_1_223 | 超时→超时 | 0 | 77 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| 1cutin74 | 超时→超时 | 0 | 10 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| highway_merge_3_1_432 | 超时→超时 | 0 | 79 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| highway_merge_3_1_495 | 超时→超时 | 0 | 4 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| highway_merge_3_1_18 | 超时→超时 | 0 | 0 | None | route + candidate_progress + selection_budget |
| 7_661_merge_664 | 碰撞→碰撞 | 35 | 0 | None | candidate_generation + occupancy_prediction + execution_constraints; prediction + occupancy + candidate_selection |
| highway_merge_2_1_482 | 超时→超时 | 0 | 80 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| highway_merge_3_1_26 | 超时→超时 | 0 | 5 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| highway_merge_2_2_118 | 超时→超时 | 0 | 1 | None | route + controller + mapped_boundary; route + candidate_progress + selection_budget |
| 7_277_merge_292 | 碰撞→碰撞 | 18 | 0 | None | candidate_generation + occupancy_prediction + execution_constraints; prediction + occupancy + candidate_selection |
| 13_91_straight_in_opposite_left_96 | 碰撞→碰撞 | 4 | 1 | None | candidate_generation + occupancy_prediction + execution_constraints; route + controller + mapped_boundary; prediction + occupancy + candidate_selection |
| 18_29_straight_straight_30 | 超时→超时 | 35 | 36 | 4.7 | candidate_generation + occupancy_prediction + execution_constraints; route + controller + mapped_boundary; route + controller + mapped_boundary; route + controller + mapped_boundary + risk_selection; route + candidate_progress + selection_budget |
| 11_57_straight_straight_61 | 碰撞→碰撞 | 0 | 84 | None | route + controller + mapped_boundary; visibility_birth |
| 20_46_straight_in_adjacent_left_47 | 碰撞→碰撞 | 3 | 0 | None | candidate_generation + occupancy_prediction + execution_constraints; prediction + occupancy + candidate_selection |

逐候选淘汰原因、成本分项、首冲突对象/分支、执行序列及下一帧对账在每场原始压缩日志中。
首次/末次风险帧与全部失败分类见 `failure_attribution.json`。

官方回放结局保留；到达但实际车身外部面积增长的场景也进入问题表。
初始外伸单独记录，地图覆盖核查不等同于合法道路或连续扫掠认证。

## 决策链首次分歧

| 场景 | 首次动作分歧 | 占用/跟踪见证丢失 | 道路见证丢失 | 有候选但预算丢失 | 最后前缀可行决策 |
|---|---:|---:|---:|---:|---:|
| 2422cutin168 |  |  |  | 1.28 | 8.8 |
| 1lanechange42 |  |  |  | 8.9 | 14.7 |
| highway_merge_3_1_223 |  |  |  | 1.1 | 9.9 |
| 1cutin74 |  |  |  | 6.8 | 14.7 |
| highway_merge_3_1_432 |  |  |  | 0.6 | 9.9 |
| highway_merge_3_1_495 |  |  |  | 0.3 | 9.9 |
| highway_merge_3_1_18 |  |  |  | 4.7 | 9.9 |
| 7_661_merge_664 |  | 30.7 |  | 30.3 | 30.6 |
| highway_merge_2_1_482 |  |  |  | 0.7 | 9.9 |
| highway_merge_3_1_26 |  |  |  | 2.6 | 9.9 |
| highway_merge_2_2_118 |  |  |  | 3.8 | 9.9 |
| 7_277_merge_292 |  | 40.1 |  |  | 40.0 |
| 13_91_straight_in_opposite_left_96 |  | 10.8 |  |  | 10.7 |
| 18_29_straight_straight_30 |  | 2.9 | 4.4 | 8.0 | 19.7 |
| 11_57_straight_straight_61 |  |  |  |  | 9.8 |
| 20_46_straight_in_adjacent_left_47 |  | 11.0 |  |  | 10.9 |

表中候选见证来自当帧已保存的评估，不是反事实闭环结果；道路未查、预测支和成本分项均在JSON中保留。
