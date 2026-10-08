# OnSite ARB＋W1：r99 开发底座

当前版本为 `ARB-CAPABILITY-recovery_pulse_r99_devreg130`。本仓库保存 r99 封存源码、从输入到控制的架构解释、学习指南及完整本地验证证据。整理日期：2026-10-08。

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

当前处于暂停状态。继续开发应从封存版本复制新候选，保留原成功且不引入新增碰撞，不能用净到达增益抵消回归。
