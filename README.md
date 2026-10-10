# OnSite ARB＋W1：r99 底座与 r43 统一执行候选

2026-10-10更新：新增 **r43统一执行候选的34文件源码、230场结果、全车身道路审核、31场未到达归因，以及与已上传r99的逐场/逐文件比较**。r43提高净到达数，但有成功损失、新碰撞和时延退化，尚未替换r99。根目录 `stack/` 和原r99归档继续保持原样。

最新内容从 [版本比较](docs/UPDATE_20261010.md)、[候选架构](docs/UNIFIED_EXECUTION.md)、[候选复现](docs/REPRODUCTION_R43.md) 和 [全部未到达归因](candidates/r43/analysis/nonarrival_attribution_r59/v2/REPORT.md) 开始。

| 同一原130场 | 已上传r99 | 当前r43候选 |
|---|---:|---:|
| 到达／碰撞／超时 | 107／7／16 | 115／3／12 |
| 逐场完整act时延通过 | 130／130 | 72／130 |
| 相对r99成功保持 | — | 103／107；新增12到达、丢4成功 |
| 相对r99新增碰撞 | — | 1：7_916_merge_921 |

场景名单和原始文件哈希一致；r43明确在内存中排序7场反序目标框，XML未改，这部分协议差异不能全部算作策略收益。其余123场为107／7／9→114／3／6。额外100场是追加开发集，结果84／5／11；不能与r99原130混成同分母比较。合计230场199／8／23，仍有4场实际车身地图外伸增长。

```powershell
python tools/verify_snapshot.py
python tools/compare_uploaded_versions.py
python tools/replay_r43.py --help
```

`verify_snapshot.py`校验上传包、原r99身份及候选源码/结果/已包含trace；`compare_uploaded_versions.py`只读证据，输出到忽略目录outputs；以上不启动驾驶。候选生产源码逐字节保留，不混入否定实验。r43只有3场原trace上传，其余哈希和大小保留，详见候选复现说明。

以下是仍然有效的r99底座说明，架构和学习材料以r99为对象。

结构化观测 → 周车运动估计与 W1 预测 → 车道图路线及换道时机候选 → 六秒速度候选仲裁 → 两秒实际执行复核 → 路径跟踪与控制 → 官方回放反馈。

## 从这里开始

- [全链条架构说明](docs/ARCHITECTURE.md)：实现、公式、数据流、参数和已知缺口。
- [学习指南](docs/LEARNING_GUIDE.md)：按具体场景理解路线、轨迹、动作与闭环。
- [复现与证据说明](docs/REPRODUCTION.md)：校验方法、运行入口和外部依赖。
- [暂停状态](docs/PAUSED_STATUS.md)：当前成效与未完成问题。
- [时空占用教学示意](docs/onsite-time-occupancy.html)：下载后用浏览器打开；示意数字是假设值。

## 当前结果

| 指标 | r99 |
|---|---:|
| 完整 DEV100＋REG30 场景 | 130 |
| 到达／碰撞／超时／异常 | 107／7／16／0 |
| 到达率 | 82.3% |
| 相对 r96 | 保留全部104成功，新增3到达，无新增碰撞 |
| `act()` 次数 | 28,659 |
| 平均／p95／p99 | 5.8206／9.2740／14.9521 ms |
| 最大调用耗时 | 82.0291 ms |
| 逐场时延审核 | 130／130通过 |

计时包含完整 `act()`，不含初始化、台架和记录 I/O。逐场通过不代表每次调用均小于周期；记录存在一次超过对应 `dt` 的调用。共享负载诊断不等于独占机器或官方实时认证。结果来自复用官方回放核心的本地闭环台架，不是官方线上比赛成绩，现存失败没有被隐藏。

## 目录与版本身份

```text
stack/                       与封存 r99 逐文件相同的可导入源码
docs/                        架构、教学、复现、暂停与历史决策
tools/verify_snapshot.py      无需仿真的离线完整性校验
tools/replay_r99.py           外部依赖准备后可用的隔离回放入口
bench/                       r99 运行归档内的本地台架源码
baselines/arb_w1_v1/          v1 冻结源码、清单与130场记录
research/arb_capability_20261007/
  runs/recovery_pulse_r99_devreg130/     r99封存源码、原始记录和审核
  runs/dynamic_route_r5_devreg130/       r5参考证据
  runs/supported_pruned_r96_devreg130/   r96参考证据
EXPORT_MANIFEST.json          上传包文件哈希与来源
```

正式入口是 `stack.adapters.onsite_close_guard:CloseGuardPlanner`。根目录 `stack/` 是 r99 的精确副本；原工作区尚未切换默认入口。本仓库不包含旧架构的全部开发历史，也不包含第三方工具包、官方场景原始数据、地图缓存、凭据或机器环境。平台和场景的原始身份仍在验证 manifest 中保留；重新跑仿真需按复现文档另行准备。

```powershell
python tools/verify_snapshot.py
```

原r99归档保留。继续开发应从封存版本复制新候选，保留原成功且不引入新增碰撞，不能用净到达增益抵消回归。本次GitHub更新没有恢复已停止的半小时算法实验。
