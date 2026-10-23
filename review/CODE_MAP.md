# 四模块与具体源码入口

原 R43 的唯一源码根是 `candidates/r43/source/`；34 文件保持提交 `1dbd050` 的原字节。不要把原型路径加入同一个 Python 导入搜索路径。

| 模块 | 原 R43 入口 | 审阅重点 |
|---|---|---|
| 预测 | `stack/core/world.py`、`stack/core/unified_evaluation.py` 的 `ExecutionEvaluator` | 因果历史、W1/CV/支持加速的启用时域、膨胀矩形占用 |
| 决策 | `stack/core/unified_evaluation.py` 的 `execution_rank` / `select_execution`；`stack/adapters/onsite_unified.py` | 期限如何参与选择，完整窗与两秒硬窗的区别，无可行项时的风险缓解 |
| 规划 | `stack/core/candidates.py`、`stack/core/compound_route.py`、`stack/core/tail_eta.py`、`stack/adapters/onsite_dynamic_route.py` | 持续常量与脉冲、有限换道时机、曲率速度包络、窗外 ETA |
| 控制/执行 | `stack/core/control.py`、`stack/core/native_execution.py`、`stack/core/replay_contract.py` | 转向记忆、官方下一步模型、三位小数舍入、实际模型展开与命令 |

表中的 `stack/...` 均相对各独立源码根，不是根目录的另一份默认算法。

## A 轮

源码：`experiments/selector_a/source/`。仅两个原源码文件变化：`stack/core/unified_evaluation.py` 与 `stack/adapters/onsite_unified.py`。完整差异见 `patches/r43_to_selector_a.diff`。

核心是保留原选择作为比较，在全部启用预测/道路窗口合格项中优先按时和较早 ETA。它没有解决所有候选家族一致遵守速度包络的问题。结果与四场真实轨迹同目录保存，不把原成功回归掩盖在平均收益里。

## 最近失败原型

源码：`experiments/gap_event_v2/source/`。它继承前轮整体合流与原模型速度序列，不是单独只改 before/after 的候选。完整差异见 `patches/r43_to_gap_event_v2.diff`。

- `stack/core/gap_events.py`：`forecast_events` 提取 actor-based 事件，`_event_profile` 通过原模型生成，`generate_gap_events` 追加有限候选。重点检查其终端事件实际是否满足。
- `stack/core/progress_profile.py`：原包络条件下的推进序列。
- `stack/core/compound_route.py`：连续多车道合流的整体路线。
- `stack/core/candidates.py`：事件与 `original_reference` 标签及 continuation 的继承。
- `stack/core/unified_evaluation.py`：`select_original_execution`、`select_execution` 与 `executed_speed_gate`，区分原 fallback 与新增方案的完整窗准入。
- `stack/adapters/onsite_unified.py`：新路线承诺后的集成与失解时拒发动作。`RuntimeError` 是保存的算法停止，不是驾驶成功。

`test_gap_events.py` 与 `test_selector.py` 为历史测试源码原件；测试通过的范围以各目录的历史 verification 为准。本次未重跑机制测试或驾驶。
