# 原版 R43 冻结比较底座

`source/` 的 34 个 Python 文件逐字节保持提交 `1dbd0507353666c90ec7a9fdc3c752b62e85d001`。入口是 `stack.adapters.onsite_unified:UnifiedPlanner`，独立模块根为本目录的 `source/`。不要与两个实验原型混合导入。

`verification.json` 和两个 `runs/` 下的运行身份、源码 ZIP、结果与审核为历史原件。原 130 场 115 到达 / 3 碰撞 / 12 超时，追加开发 100 场 84 / 5 / 11，合计 199 / 8 / 23。该版存在碰撞、超时及道路外伸，不是安全验收完成版。

本次审阅保留原始源码，不将任何后续修复写回底座。两份 run 中共有 7 场原始压缩 trace；全部 230 场 trace 哈希与大小仍在 `trace_index.json`。旧报告中的绝对路径、曾经的比较和导出范围是当时记录，不代表本次包的目录或完整性。

当前问题从根目录 `review/REVIEW_BRIEF.md` 开始；当前导出范围、校验和外部依赖见根目录 `review/EVIDENCE_INDEX.md` 与 `review/REPRODUCTION.md`。本次身份以根目录 `REVIEW_MANIFEST.json` 为准。
