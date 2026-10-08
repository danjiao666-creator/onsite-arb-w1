# 当前开发基准

当前已暂停。最新通过的开发底座更新为 `research/arb_capability_20261007/runs/recovery_pulse_r99_devreg130/source/`，入口 `stack.adapters.onsite_close_guard:CloseGuardPlanner`。完整130场107/7/16、0异常，保留r96全部104成功，无新增碰撞；身份、双基准、驾驶与逐场时延130/130通过。默认未切换，23个失败仍待解决。以下r96段落保留历史说明，最新证据见 `research/arb_capability_20261007/PAUSED_STATUS.md`。

决策日期：2026-10-07。用户明确要求先形成一版性能较好、能力较强的版本，此后所有版本依靠它调优。

当前后续开发从 `research/arb_capability_20261007/runs/supported_pruned_r96_devreg130/source/` 出发，入口 `stack.adapters.onsite_close_guard:CloseGuardPlanner`。完整130场104到达、10碰撞、16超时、0异常，保留r92全部103成功，无新增碰撞。新增10_441到达；周车加速分支只作用于估计加速度非负的车辆，避免对正在制动车辆强加加速假设。源码、输入、v1/r5双基准审核及驾驶逐场配对通过；本次逐场时延130/130通过。共享负载观测不作为独占或官方验收。默认未替换，固定v1/r5及历代通过候选继续封存。详细证据见 `research/arb_capability_20261007/CURRENT.md`、r96的 `release_review.json` 和 `parent_comparison.json`。r97完整106/8/16但新增14_132碰撞，拒绝晋级。

## 唯一基准与当前边界

固定基准为 **ARB-W1-BASE-v1**，源码在 `baselines/arb_w1_v1/source/`，恢复包为该目录的 `sources.zip`，逐文件 SHA256 在 `manifest.json`。它取自已完成近期十二场比较的 ARB+W1 归档，不依赖之后可变的 `stack/` 源码。冻结的是算法和 DEV100/REG30 清单；回放平台、环境和每次输入另行记录，不能把算法快照称为全环境镜像。

本版本是性能和驾驶能力的固定比较起点，仍有已知碰撞与超时，不是安全验收完成版。本轮隔离入口完整重跑130场，结果为 **72到达、24碰撞、34超时、0运行异常**。26,913周期的实际状态和动作与历史ARB记录全部相等，来源和输入前后不变。结果位于 `baselines/arb_w1_v1/validation/devreg_20261007_r1/`，逐帧核验与性能明细见 `release_audit.json`。

本轮完整act合并统计：平均16.56ms、p95 28.70ms、p99 38.69ms、最大176.71ms；**逐场周期目标110/130通过，20场未通过**。合并分位数不能替代逐场验收，未将该版称为完整实时达标。失败场景完整保留，是后续性能优化清单。

运行默认 ARB+W1 保留。轻量版 v36/v37 不再是开发主线，旧证据及道路、执行、碰撞等有用组件保留。`MultiFeasible` 是后续增强候选：历史130场76到达、23碰撞、31超时，保留原72道成功，但有一场新增碰撞，尚不晋级为默认或替换冻结基准。候选归档为 `research/architecture_takeover_20260926/feasible_multi_r1/sources.zip`。

## 后续版本如何调优

1. 每个候选从明确的源码快照出发；先改变一个有证据的能力缺口。第一优先是路径与速度候选的表达和生成，Panda 的路径优化、ST 速度规划可作组件来源。
2. 每轮比较使用相同场景、输入内容、车辆和评价口径；同时报告到达、碰撞、超时、运行错误、逐题成功丢失/新增碰撞、共同成功的完成时间及舒适性。DEV/REG 是开发和回归数据，HOLD 继续封存。
3. 完整 `act()` 包含异常调用，报告 p95、p99、最大、首调用和超时比例。当前周期目标为每场 p95 ≤ min(30ms, 0.75dt)、p99 ≤ min(40ms, dt)、超 dt ≤ 1%；并发跑题耗时不算串行验收。未覆盖场景不得由少数场景外推通过。
4. 原成功丢失、新增碰撞或周期恶化必须逐题说明；不以总到达提升抵消。候选未通过配对检查不切默认，不改写基准失败记录。实质升级形成新版本，保留 v1 及升级链，不覆盖旧基准。

已知检查或预测缺口不能通过撤销评估来隐藏；新增检查也应先验证物理与平台含义，防止地图微缝之类伪拒绝。没有可行候选时的风险缓解仍算风险，不能写成安全通过。完整动态安全与官方评分未被此基准声称解决。

## 使用

在项目根目录使用 `C:/Users/dingj/miniconda3/envs/onsite38/python.exe`：

```powershell
python bench/run_baseline.py verify
python bench/run_baseline.py run --output baselines/arb_w1_v1/validation/NEW_RUN
python bench/run_baseline.py restore --output NEW_SOURCE_DIRECTORY
python bench/compare_baseline.py --baseline baselines/arb_w1_v1/validation/devreg_20261007_r1/results.json --candidate CANDIDATE/results.json --output COMPARISON.json
```

`run` 默认完整130场、单进程；定向检查可用 `--scenes name1,name2`。`restore` 只写不存在的新目录，不覆盖工作区。冻结文件校验不通过时先查明修改来源，不重新生成清单来掩盖变更。比较工具拒绝缺失或重复场景，不自动宣布晋级。

## 复现资产与检查结果

完整运行保存130份逐题状态/动作/计时压缩日志、输入哈希、解析器和台架源码哈希及副本 `runtime_sources.zip`、可用地图缓存副本 `map_cache.zip`、环境包清单 `requirements-freeze.txt`。首轮地图缓存只在结束后归档，原缓存生成时的解析器身份未知，不能声称首轮已核验缓存前后完全不变；最终运行入口已补运行前后的缓存身份与归档，并通过单场行为等价检查。

恢复入口已验证：展开源码损坏可被发现，完整ZIP仍能恢复到新目录，恢复后逐文件哈希一致，已有目录不能覆盖。比较器8项检查通过，真实130场历史配对无新增/丢失成功、无新增碰撞。工具修正未改冻结规划器；最终入口定向复测与完整运行对应场景逐帧相等。
