# r43完整230候选审核

2026-10-10 06:23，原130和归一扩100候选完整驾驶、实际道路、逐帧归因、源码/输入/trace/工具哈希、合同和总审均完成；冻结r26驾驶未重跑。

| 开发集 | 冻结r26 | r43 | 动作/状态等价 | 时延通过 |
|---|---|---|---:|---:|
| 原130 | 114到达/3碰撞/1出界/12超时 | 115到达/3碰撞/12超时 | 129/130 | 72/130 |
| 归一扩100 | 84到达/5碰撞/11超时 | 84到达/5碰撞/11超时 | 100/100 | 56/100 |

full_candidate_driving_comparison_passed=true：0新增碰撞、原成功损失、算法异常、输入失配、执行失配、实际道路退化和新增外部增长。只14_14从22.4秒出界变12.0秒到达、二次外伸消失；其初始外伸0.9秒回图，对照0.7秒，不称每项时间指标改善。全230仍8碰撞/23超时，道路增长保留12_60/14_123/8_282/18_29四场；absolute_mapped_body_grid_passed=false，timing_fully_passed=false，默认不晋级。旧路线父版的4成功丢失和7_916新增碰撞也仍存在，这是r26已有退化。

源码相对r26仅统一入口接入与新增stack/core/compound_route.py，其余32文件保持。主/恢复/续行评估、道路门、buffer、成本、预测和单一ExecutionDecision不变。进入连续过渡后保持所选几何及全部图边承诺。两完整run的34源文件哈希相同，归档因ZIP时间元数据哈希不同，不当源码差异。

证据：full230_review.json SHA256 d76e3c9b98df792e3ce08d4abd854e8d610b9fad97e2038470ddffe1182ba540；完整run位于../../runs/unified_r43_20261010_devreg130、unified_r43_20261010_extra100。随run冻结的diagnostic_tools及实际道路报告绑定manifest/trace；原130对r26配对在comparisons/unified_r26_20261009_devreg130，扩100在run根目录。失败/道路问题均记录首失效帧和候选、对象分支、预算、控制及模块耦合，到达但外伸照样列问题表。

最新verification SHA256 1e561680d4b30a71a49dc66b140b4b5fb91739b6bc9e48d4e7d611e9c70dc8d1，当前64和封存r32历史35合同均过，公共27方法AST未改。初始筛选wrapper及verification保留analysis/r43_source_integration/initial_contracts，两run的当时验证不覆盖；r32完整源/测试/旧工具保留rejected_r32。新回归只写允许目录，工具支持绝对基准定位。

下一修复：8_282的4.2秒冷启动预测支持与道路内可恢复动作。现有源以accel>=0启用1.5m/s²扩张，car10初见accel=0且没有成熟误差，321/381原候选由此支拒绝；但255条真实记录前缀接触，不能直接删分支。道路内21条W1/CV无前缀冲突仅为有限探针；未来不进入选择。支持审计../prediction_support_r43/8_282_t4p2_support.json SHA256 ee8042da8c89ace4125dca45a2e7ad6969cc70ea6e363e772980eecc2270c07b。随后7_661和扩集7_277/car6、13_91/car8、20_46/car2；保留r43收益，先关键配对与实际道路再有收益全量。

所有Python驾驶/报告PID与相关exec已结束，正常等待安静，自动化继续本线程。不可改默认、XML、原名单、冻结run或读取/运行HOLD50内容；道路几何检测不等同法律违例，回图后再次外伸不得用起点面积掩盖。
