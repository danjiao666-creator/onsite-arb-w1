# r43源码、结果与复现范围

## 无驾驶验证

```powershell
python tools/verify_snapshot.py
python tools/compare_uploaded_versions.py
python tools/replay_r43.py --help
```

先校验字节级EXPORT_MANIFEST，再校验原v1/r5/r96/r99源码与旧证据、根stack仍等于r99；随后校验r43/两run源码、ZIP、名单、结果、道路报告和已包含trace。比较只读取原130的两版results，不使用额外100混同分母，输出到 `outputs/version_comparison/`。

Windows请保持原字节；本仓库新增 `.gitattributes` 防止换行转换破坏历史哈希。原机器绝对路径仅为历史身份；程序应使用当前检出路径。

## 上传证据范围

r43所有34文件、两run的source/ZIP/sets/manifest、全部230结果、完整逐场比较/模块/实际道路报告及31场归因均包含。原始trace精确复制3场：原130的14_14与7_916、追加100的20_46。其它227场压缩trace共约520MB仍在本地，trace_index和EXPORT_SCOPE保留其SHA256/大小；平台runtime ZIP和地图缓存同样未上传。哈希证明导出身份，不能凭未上传文件声称此仓库含完整轨迹或已重算全部交通/道路接口。

本地原归档为 `research/arb_capability_20261007/unified_execution_20261009/runs/unified_r43_20261010_devreg130` 和 `unified_r43_20261010_extra100`。本仓库选用较短的 `candidates/r43/` 导出路径；不影响源码身份。所有报告原件保留，可能引用未导出文件或旧机器路径。

## 合同验证

27项统一执行、4项路线接缝、6项连续横移和3项实际道路测试可直接用候选source执行；24项P0测试另外需要官方工具 `utils/functions.py`，只提取两个纯函数，不导入其日志模块，不上传第三方源码。

```powershell
python tools/check_r43_package.py --output outputs/r43_package_check.json
python tools/check_r43_package.py --official-functions C:/path/to/onsite-structured-test-3.1.5/utils/functions.py --output outputs/r43_package_check_with_official.json
```

无官方依赖时输出明确记录P0未运行，不把40项当完整64项。这些是合同/打包验证，不是驾驶或安全验收；不覆写已有verification。

## 单进程回放

需要Python3.8兼容依赖：numpy、numba、shapely、opencv和官方3.1.5平台解析器等；仍沿原 [外部依赖说明](REPRODUCTION.md)。台架bench和architecture_takeover在r43运行runtime归档中逐字节等于原GitHub版本。

```powershell
python tools/replay_r43.py --output new_runs/r43_target --scenes 14_14_left_straight_in_adjacent_38
python tools/replay_r43.py --output new_runs/r43_extra --set extra100 --scenes 20_46_straight_in_adjacent_left_47
```

入口明确导入对应冻结run/source、校验源码/ZIP/名单，按run原输入协议在内存排序目标边界并记录，拒绝策略环境覆盖。输出须为 `new_runs/` 内的新目录，默认完整原130；额外开发集需明确 `--set extra100`。HOLD不包含、不运行。启动驾驶前校验所选原输入和全部平台/台架源码与冻结manifest一致，结束后核对完整输入清单。新回放保留每步候选与执行反馈；完整道路的历史审核已上传，新回放的道路报告需外部地图工具另行生成，不以仿真结局自动证明全车身道路通过。

本次上传只验证导出和入口，没有执行新驾驶，未改变本地默认、XML或冻结run。
