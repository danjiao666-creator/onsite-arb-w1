# 复现与证据说明

本文保留r99复现约定。2026-10-10新增r43候选的入口与导出范围见 [REPRODUCTION_R43.md](REPRODUCTION_R43.md)；r99源码/原始轨迹继续保留，根目录stack未替换。

## 离线验证

在仓库根目录执行：

```powershell
python tools/verify_snapshot.py
```

只读取文件，不启动规划器或仿真。验证上传包哈希、四个快照的源码/ZIP/场景清单、根目录stack与r99身份、130场结果/日志数量、r99审核中的原始日志哈希和调用数。

`EXPORT_MANIFEST.json`是本次上传包的字节级清单；各快照原有manifest是运行时源码身份，不能互相替代。记录中的旧机器绝对路径是历史证据，不作为当前可执行路径。

## 外部依赖

回放需要Python3.8环境及OnSite结构化测试工具3.1.5的兼容依赖（NumPy、Numba、OpenCV和平台解析器等）。原环境解释器为 `onsite38`。完整平台与环境不能仅由本仓库推断，准备依赖时以运行manifest的环境版本为准。

按如下位置放入外部资产：

```text
code/onsite-structured-test-3.1.5/   官方回放工具及其依赖
scenes/official_catalog/extracted/<场景名>/   xodr、xosc等原始输入
scenes/official_catalog/index.json   本地台架所需场景目录索引
```

这些资产未上传。r99的 `validation/manifest.json` 保留原输入和平台源码哈希；必须比对后才能称同输入/同平台复现。地图缓存未上传，首次地图解析可能更慢。原运行runtime_sources.zip含第三方工具源码，未上传完整ZIP；本仓库台架从该ZIP逐字节提取，只保留本地bench和runner部分。

## 回放入口

当前用户暂停实验；以下是以后恢复时的命令，本次上传没有执行它。

```powershell
python tools/replay_r99.py --output new_runs/r99_replay
python tools/replay_r99.py --output new_runs/r99_target --scenes 12_97_straight_straight_100
```

入口先校验封存r99，然后将其作为`run_baseline`的独立BASE和ENTRY运行；输出必须是不存在的新目录。默认完整DEV100+REG30、单进程，拒绝ARB_/EXEC_策略覆盖。根目录stack虽然同r99，入口仍强制从封存source导入并检查路径。

## 证据目录

- r99完整130场原始压缩轨迹、动作、完整act计时、results、summary、verification、release_review和parent_comparison。
- r5、r96完整130场对应证据；v1为`baselines/arb_w1_v1/validation/devreg_20261007_r1/`。
- 封存源码、sources.zip、sets.json和manifest原样保留。
- 审核引用的平台和地图缓存哈希保留，但本仓库没有这些被排除的文件，不能声称完整环境镜像。

到达/碰撞/超时是官方回放核心的终止结果；score是本地评分；latency是完整act计算耗时。三者不得混称官方线上成绩。
