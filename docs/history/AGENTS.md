# 项目事实与当前工作方向

- 用户已要求暂停。最新完整审核通过的开发底座为 `research/arb_capability_20261007/runs/recovery_pulse_r99_devreg130/source/`，入口 `stack.adapters.onsite_close_guard:CloseGuardPlanner`，130场107到达/7碰撞/16超时、0异常，保留r96全部104成功，无新增碰撞；身份、v1/r5双基准、驾驶及逐场时延130/130通过。默认未切换。暂停状态与最新性能见 `research/arb_capability_20261007/PAUSED_STATUS.md`；下方r96及更早版本均是历史记录。用户恢复前不再启动实验。

全局沟通和执行规则沿用用户提供的 canonical AGENTS.md，此处只记录项目事实。

- 用户于 2026-10-07 确认：以 ARB+W1 为改造底座，先形成一版性能较好、能力较强的基准，此后所有版本基于它调优。
- 当前基准、边界、复现命令和晋级要求以根目录 `BASELINE.md`、`baselines/arb_w1_v1/manifest.json` 及验证原始结果为准。冻结源码不得原位编辑；从独立候选副本开发。
- 不再默认沿 `research/traffic_lightweight_20260930/CURRENT.md` 的旧轻量版迭代路线推进；该文件保留历史实验与交接事实。后续主线优先改善路径和速度候选能力，有用组件可复用。
- ARB-W1-BASE-v1 是开发比较基准，不能称安全合格、完整实时验收完成或官方比赛成绩。历史数据与当前运行区分记录。候选存在已知新增碰撞时不得直接替代默认。
- 后续性能比较记录完整 act、异常、同场输入与源码身份；HOLD 继续封存。现有自动化已由用户要求删除，不恢复自动化。
- 最新通过的开发底座是 `research/arb_capability_20261007/runs/supported_pruned_r96_devreg130/source/`，入口 `stack.adapters.onsite_close_guard:CloseGuardPlanner`。完整130场104/10/16、0异常，保留r92全部103成功，无新增碰撞，源码/输入/v1/r5及驾驶配对审核通过；本次逐场时延130/130通过。默认未切换，仍有26个失败。r97完整106/8/16但新增14_132碰撞，拒绝晋级；后续从r96开发。
- 历史底座为 `research/arb_capability_20261007/runs/exact_envelope_r53_devreg130/source/`，入口 `stack.adapters.onsite_cadence:CadencePlanner`；完整130场97/14/19、0异常，与r34全部27,453周期严格等价，本次时延130/130通过。共享负载观测不等于官方认证，默认未切换。r60-r72为独立机制实验，未经完整成功保持及无新增碰撞审核不得合并，结果见 `CONTINUATION_20261007.md` 及各run的 `parent_comparison.json`。
- 历史已完成并审核 `prepared_eta_r78_devreg130`：入口同上，完整130场99/14/17、0异常，保留r53全部97成功且无新增碰撞；与预算修复r77全部27,466周期严格等价。此次逐场时延130/130通过，共享负载边界不变。只修正预算ETA的末速/弯道外推与编译预热，v1/r5/r53保留，默认未切换。具体见`CURRENT.md`、`prepared_eta_equivalence.json`与r78的`release_review.json`。
- 2026-10-07 首轮根因修复已形成 `ARB-CAPABILITY-dynamic_route_r5_devreg130`：130 场 76 到达/22 碰撞/32 超时、0 异常，保留原 72 成功且无新增碰撞；逐场时延仅 108/130 通过。后续能力开发入口、封存源码和剩余问题以 `research/arb_capability_20261007/CURRENT.md` 为准，仍与 v1 固定基准配对；该候选未切默认，不得称全部实时或安全达标。
- 同日后续能力实验 `goal_box_r19_devreg130` 完整130场为86到达/14碰撞/30超时，保留r5全部76成功、无新增碰撞或异常，双基准身份与回归审核通过；共享负载时延仅59/130通过，未切默认。用户指出此前仍未打到根本：局部修复收益不得作为核心可行性模型可靠的证据；后续优先核验自车执行占用、周车未来占用的假安全/假无解及早期可行解丢失，具体边界见 `research/arb_capability_20261007/FUNDAMENTAL_REVIEW.md`。
