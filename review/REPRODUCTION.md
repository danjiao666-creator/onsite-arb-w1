# 离线核验与复现边界

## 无外部依赖即可完成

Python 3.8 及以上，标准库：

```powershell
python -B tools/verify_review.py
python -B tools/unpack_replay.py
```

核验脚本检查包内文件哈希、原 R43 34 文件与原 verification 的一致性、两个原型与各自冻结源码哈希的一致性、230 场结果统计、当前失败事实，以及压缩回放解压后原始哈希。脚本不 import 算法、不求解、不调用仿真。结果写 stdout，不修改历史收据。

解压工具默认输出 `outputs/r43_all_failures.html`，已存在则拒绝覆盖；该目录被 Git 忽略。四个较小 HTML 自包含，可直接用浏览器打开，不依赖本机旧端口。

## 本次没有声称可一键重跑仿真

原运行使用 Python 3.8、numpy 1.19.5、numba 0.56.4、shapely 2.0.7 和外部官方回放核心。官方工具、场景输入、地图缓存及其原版本需要另行准备并对照历史 manifest。此包不包含官方平台、全部 230 场 trace 或整个本机环境。

原始报告/manifest 的绝对路径、排除条目与哈希作为历史原件保留，不能直接当成当前机器上的有效文件位置。原算法源内的默认路径也未为打包而偷偷改动。原型 orchestration 脚本、旧自动化状态和历史 protected 文件库存不作为可运行入口导出。

`REVIEW_MANIFEST.json` 是本次发布身份；各目录的旧 manifest / verification 是当时运行身份，两者不可混用。本次未重新计算驾驶成绩、未重跑机制测试，也未将打包校验称作安全验收。
