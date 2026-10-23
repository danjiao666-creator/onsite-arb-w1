# R43：合流超时与规划可行性审阅

**希望审阅的不是再加一条规则，而是：现有候选表达、选择目标和滚动规划，为什么不能稳定地在期限内安全完成合流？最小的有效替换应在哪里？**

2026-10-23 整理。底座固定为 [`1dbd050`](https://github.com/danjiao666-creator/onsite-arb-w1/commit/1dbd0507353666c90ec7a9fdc3c752b62e85d001) 的原版 R43，不是后来的 201 到达版本，不包含联合 MPC 工作线。当前文件树撤下 R99、旧基准和无关历史说明；原历史提交不删除。

## 先看这三处

1. [当前问题、证据和请审阅的问题](review/REVIEW_BRIEF.md)：先把事实与假设分开。
2. [四模块与改动入口](review/CODE_MAP.md)：原 R43、A 轮、最近失败原型分开读。
3. [证据与回放索引](review/EVIDENCE_INDEX.md)：全部 23 超时，以及 112 / 68 / 332 的真实执行和未提交候选。

## 目前结果

| 对象 | 已保存的结果 | 不能据此声称 |
|---|---|---|
| 原 R43 | 230 场：199 到达、8 碰撞、23 超时；其中 18 个合流超时 | 安全或实时验收完成 |
| A 轮：完整窗准入与 ETA 排序 | 68 / 332 分别在 6.5 / 6.6 秒到达；原成功 112 变超时；两恢复场峰速约 50.487 / 43.950 m/s | 已得到可接受的稳定修复 |
| 后续五个有限候选 | 统一速度包络、条件救援、有界速度序列、整体合流及组合；没有新增整场恢复 | 更多候选已解决交通时机 |
| 最近 gap-event v2 | 112 保持 9.7 秒到达；68 仍超时；332 实际执行 33 步，在 3.3 秒无完整合格候选而中止 | 起点 ETA 合格等于完整可行 |

最近原型的 `before/after` 仍是生成目标，**不是已落实的硬站点时刻约束**。332 初始有两条有限窗合格方案，但执行中遇到后方变道的 car1 后失解。此次没有新增算法、驾驶、求解或自动化。

## 文件结构

```text
candidates/r43/source/           原 R43 的 34 个冻结源码
candidates/r43/runs/             原 230 场结果、身份与选定原始轨迹
experiments/selector_a/          A 轮源码、结果与四场真实轨迹
experiments/gap_event_v2/        最近失败原型、三场真实轨迹与失败调用数组
patches/                        两个原型相对纯 R43 的源码差异
evidence/                       四模块、全部超时、五候选冲刺的已存证据
replays/                        自包含 HTML 回放；全部失败回放使用 gzip 压缩
review/                         审阅问题、代码地图、证据范围和来源
tools/verify_review.py           标准库离线校验，不导入规划器、不驾驶
tools/unpack_replay.py           解压全部失败回放到 outputs/
REVIEW_MANIFEST.json             本次审阅包文件哈希
```

```powershell
python -B tools/verify_review.py
python -B tools/unpack_replay.py
```

解压后用浏览器打开 `outputs/r43_all_failures.html`。其他回放可直接打开：[最近失解](replays/gap_event_v2.html)、[A 轮对照](replays/selector_a.html)、[全部超时](replays/all_timeouts.html)、[五候选冲刺](replays/merge_sprint.html)。GitHub 文件页本身不执行 HTML，需下载仓库后打开。

原始场景、官方平台和地图缓存不在包内；这里提供可独立阅读的代码、保存证据和回放，不声称下载后即可重跑完整仿真。详见[核验与复现边界](review/REPRODUCTION.md)。
