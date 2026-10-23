# 证据与回放索引

## 推荐查看顺序

1. 打开 `replays/gap_event_v2.html`，默认 332；选“car1占用失解 · 3.3s”。橙线是未提交候选，实际驾驶止于 3.3 秒。看预测分支，不把真实未来当在线输入。
2. 同页切 112，确认 97 步、9.7 秒到达保持；切 68，确认 100 步原行为未改、仍超时。
3. 打开 `replays/selector_a.html`，看 68 / 332 的两个真实恢复，再看 112 回归和速度峰值。
4. 打开 `replays/all_timeouts.html` 看全部 23 超时；`evidence/timeout_audit/ALL_TIMEOUTS.md` 提供逐场文字与候选原因。17 个高速合流加 1 个其他合流，共 18 个合流超时。
5. 如需四模块全部失败，执行 `python -B tools/unpack_replay.py`，打开 `outputs/r43_all_failures.html`：31 个终局失败加 3 个到达但道路外伸事件，含因果重建预测。

## 可直接核对的原始记录

| 位置 | 内容 |
|---|---|
| `candidates/r43/runs/*/validation/results.json` | 原 230 场逐场结果：130 场与追加开发 100 场，均非 HOLD |
| `candidates/r43/runs/*/trace_index.json` | 原全部 trace 的哈希/大小，不表示全部 trace 已上传 |
| `candidates/r43/runs/*/validation/*.json.gz` | 本包 7 场原始 trace：既有 14_14、7_916、20_46，新增 112、141、68、332 |
| `experiments/selector_a/driving/*/trace.json.gz` | A 轮 4 场实际驾驶原始记录 |
| `experiments/selector_a/review.json`、`saved_diagnosis.json` | 两恢复、一对照回归、加速脉冲与峰速证据 |
| `evidence/merge_sprint/` | 五个后续有限候选的结果，以及 C2 的 car4 / car7 占用反证 |
| `experiments/gap_event_v2/driving/*/trace.json.gz` | 最近原型 3 场实际轨迹；不是完整回归 |
| `experiments/gap_event_v2/diagnosis.json` | 原成功保持、起点资格、事件语义缺口和 3.3 秒失解 |
| `experiments/gap_event_v2/failed_selection_3_30.json/.npz` | 保存观测因果恢复的 3.3 秒失败调用；不是新增驾驶 |
| `experiments/gap_event_v2/failed_selection_recovery.json` | 历史 33 帧精确恢复和 0 新驾驶的收据 |
| `evidence/failure_atlas/r43_diagnosis.json`、`r43_layers.json` | 原 34 事件与四模块证据，保留原来源路径和 SHA |
| `review/EXPORT_PROVENANCE.json` | 本次选定导出的源路径、哈希和压缩说明 |

## 读数边界

- `verification.passed=true` 在历史收据中表示该收据的证据核验通过；`algorithm_accepted=false` 没有被改成 true。
- 原三场 key trace 加本次四场为 7 场，不是上传了完整 230 场逐帧日志。全部超时审计和全部失败可视化是已存原日志的派生证据。
- 旧 `RESULT.md` 中“下一步”“不推送”“当前服务端口”等是当时状态；本次用户只授权发布审阅包，不据此重启任务。新入口统一以本目录和根 README 为准。
- 保存模型、未提交候选、真实执行和事后真值核对分开标注。六秒预测合格加无交通尾 ETA 不能宣称完整任务已安全完成。
- 原样的取证报告可能包含本机绝对路径或旧版本名称；它们只用于来源定位，不代表包内保留了旧底座或第三方环境。
