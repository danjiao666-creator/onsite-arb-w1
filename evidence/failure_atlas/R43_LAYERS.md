# 纯 R43 四模块拆解与失败证据

只讨论已提交 GitHub 的纯 R43；不混入目标关联修复后的201，也不混入联合MPC。原230场为199到达、23超时、8碰撞。31失败均已有真实记录；实际道路外伸另有4场，其中3场最终到达，不能被“到达率”掩盖。

## 身份核对

- 仓库： https://github.com/danjiao666-creator/onsite-arb-w1
- R43提交： `1dbd0507353666c90ec7a9fdc3c752b62e85d001`，提交信息 Archive unified r43 candidate and compare against uploaded r99。
- 正确本地副本： `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh`；不是 E:/Onsite比赛/github_exports/onsite-arb-w1 的旧r99副本。
- `candidates/r43/source`全部34文件SHA256与冻结run manifest逐项一致。冻结archive SHA256：`efac5095d2eb0665f8389c4ed37898fd039dc21aae4d870f265b50e705813d2a`。
- 本次只读 `git ls-remote origin refs/heads/main` 返回 `f301ba2d647761756a19b4510b606182097c7144`，说明远端main已前进，不能称当前main就是R43。具体R43旧commit保存在本地；历史publication_state记录其当时远端HEAD和API核验。本次GitHub commit API返回403速率限制，不冒充本次API验证成功。
- 两份原记录：
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\runs\unified_r43_20261010_devreg130\comparison_review.json`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\runs\unified_r43_20261010_extra100\comparison_review.json`

## 预测：已看见的车与突然出现的车必须分开

WorldModel用因果历史估计速度、加速度与衰减转向率；W1六秒，当前朝向CV与有依据的加速分支前两秒启用。

已确认：
- 8碰撞中2场对象在碰撞终帧才首次出现，先前观测没有该对象。
- 其余6场最后一步启用预测已报告冲突，返回的是risk动作，不是预测宣告安全。
- 预测反馈记录下一帧误差，但不代表已校准所有分支六秒覆盖率。

尚未证明：
- 不可见对象并不证明更早采取其他动作也无法避免。
- 不能把全部碰撞归为W1预测精度不足；当前分类只定位最后一步。

源码：
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\world.py:41`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\unified_evaluation.py:197`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\adapters\onsite_unified.py:47`

## 决策：无共同可行候选时仍返回风险缓解动作

同一仲裁器先找两秒内跟踪/占用/整车道路共同合格项，优先原期限内有模型入框见证者；否则在全部候选中按接触时刻、违规时刻、余隙等排序返回feasible=False。

已确认：
- 6个已预测首步冲突的碰撞场景最终返回risk_mitigation_no_feasible_candidate。
- 停车不自动安全，回放周车仍可驶入停着的自车。
- 在线硬检查主要两秒，六秒展开不等于六秒完整安全证书。

尚未证明：
- 无现有候选可行不证明物理无解。
- 最后一步风险动作是否能靠更早路线/动作避免，仍需单独反事实见证。

源码：
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\unified_evaluation.py:104`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\unified_evaluation.py:126`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\unified_evaluation.py:169`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\adapters\onsite_unified.py:127`

## 规划：有限路线和动作模板，期限连接不足

车道图路线加有限换道起点比例0/0.25/0.5/0.75/0.95；主速度族、常加速度/延迟制动/脉冲/制动后恢复和旧后缀生成后去重；包络限制沿路速度，tail_eta估计窗外余程。

已确认：
- 23个超时均记录到no_predicted_arrival_within_remaining_budget；这是没有按时到达见证，不是唯一根因。
- 31失败中20场至少一次选中未来车身出映射道路；这不是相同数量的实际出路。
- 候选模板数量有限，车道路线/换道时机/速度恢复/交通占用相互耦合。

尚未证明：
- 不能把超时全部称为预测保守、起步不够积极或路线错误。
- tail_eta不含完整窗外交通，不能当完整按时到达证明。
- 后来的目标关联201不属于本次纯R43，不能混入。

源码：
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\route.py:75`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\adapters\onsite_dynamic_route.py:30`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\compound_route.py:22`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\candidates.py:16`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\tail_eta.py:7`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\adapters\onsite_unified.py:97`

## 控制：执行对齐不是当前首要失配证据

自定义Frenet预瞄跟踪：曲率前馈+横向/航向反馈，限幅与转角记忆；实际控制展开按原平台动作规则更新和三位小数舍入。不是MPC或LQR。

已确认：
- 31个失败module_forensics的execution_mismatches均为空。
- 两份原回归共记录4场实际mapped_body_exterior_growth，其中3场最终到达、1场超时。
- 实际首步模型一致只能排除已记录的该类失配，不能证明路线/跟踪动态总合格。

尚未证明：
- 未来选中轨迹出路不等于已经实际出路。
- 不能把后续联合MPC平滑与舍入偏差问题移植为R43碰撞原因。

源码：
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\native_execution.py:47`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\native_execution.py:65`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\native_execution.py:77`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\core\replay_contract.py:1`
- `E:\Onsite比赛\onsite_challenge\research\arb_capability_20261007\unified_execution_20261009\gh\candidates\r43\source\stack\adapters\onsite_unified.py:131`

## 碰撞必须分开的两类

- `unobserved_collision_partner_birth`：2场，`15_31_straight_straight_37`, `11_57_straight_straight_61`。
- `predicted_first_step_conflict_returned_as_risk`：6场，`18_108_straight_straight_112`, `7_916_merge_921`, `7_661_merge_664`, `7_277_merge_292`, `13_91_straight_in_opposite_left_96`, `20_46_straight_in_adjacent_left_47`。

7_916_merge_921有两个碰撞对象，但仍只算1场，所以已预测冲突为6场，不是7场。两场不可见对象是在终帧第一次进入结构化观测，不能据此证明物理不可避免。

## 实际道路外伸

- `12_60_straight_in_adjacent_left_64`：到达。
- `14_123_straight_in_opposite_left_124`：到达。
- `8_282_merge_291`：到达。
- `18_29_straight_straight_30`：超时。

## 建议逐层查看顺序（不是修复结论）

先看6个已预测首步冲突仍返回risk的碰撞：沿首次无解→停车/恢复→最终冲突检查决策和候选是否更早丢解。再单独看2个终帧新生对象，确认观测可见性契约。然后看23个超时的路线入框、首次预算丢失、道路拒绝和停后恢复；不能把全部超时归给单层。最后复核4场实际车身外伸，区分路径、跟踪、道路几何和风险返回。现有失败记录的执行失配为0，因此不先盲改底层积分或控制接口。

本次只读旧证据；无新驾驶、预测重建、优化、算法修改、HOLD读取。逐场诊断、首个风险、首个道路证据和最后一步预测对象对照保存在同目录r43_layers.json。
