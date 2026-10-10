# r43统一执行候选

源码34文件与本地冻结r43及verification逐字节一致。入口 `stack.adapters.onsite_unified:UnifiedPlanner`，需以本目录source作为独立模块根。根目录stack为r99，两版不要混合导入。

当前原130场115到达/3碰撞/12超时，归一追加开发100场84/5/11，合计199/8/23。相对r99有4成功损失和1新碰撞；时延与实际道路未全量通过，候选未晋级。[完整比较](../../docs/UPDATE_20261010.md)。

`runs/`保存源码、ZIP、结果、运行/输入身份、逐场审核和全车身道路报告。两run的trace_index记录全部230场原始trace哈希/大小，只上传14_14、7_916和20_46三场原始trace；其它大体量每帧诊断以及外部地图/平台ZIP仍保留在原本地归档，未宣称仓库含完整230轨迹。`EXPORT_SCOPE.json`逐文件标明导出与排除。

`verification.json`是2026-10-10已有合同验证的历史原件，SHA256为 `9c1efce67345686ef7a27fa0a2a53f0dd69c60a40e851616ae2a7d0271c8d10d`。p0测试仅适配外部官方纯函数位置和仓库r99轨迹路径，24个测试方法AST保持；PACKAGING.json单列导出路径变更身份。重新运行打包测试不会覆写原verification。

复现和导出范围见 [REPRODUCTION_R43.md](../../docs/REPRODUCTION_R43.md)。原始报告中的绝对路径是历史身份，不是本仓库的可执行路径。所有被否定的r44-r62策略未收入source。
