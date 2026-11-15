# 两人GitHub协作

> 历史开发记录。当前公开版的分支、安装和贡献流程见 [开发与协作](README.md)。

> **当前协作主文档 · 2026-10-27**。适用于视觉避障与转盘抓取提速两个任务；[返回工程首页](../../README.md) · [开发文档](README.md) · [贡献约定](../../CONTRIBUTING.md)。

本项目采用GitHub官方GitHub Flow：共同主分支、每项工作独立分支、提交、Pull Request审查、合并。两个人各有自己的本地工程目录，通过GitHub同步；切换分支只是切换自己的代码，不会自动改变对方电脑上的文件。

阅读顺序：首次下载看本文；定位实现看[源码导航](源码导航.md)；环境、按模块检查与交接格式看[验证与联调](验证与联调.md)。

## 一次工作需要留下什么

| 阶段 | 具体产出 |
| --- | --- |
| 开工 | 在 Issue 或现有任务中写目标、负责人和验收条件；简单文档修改可直接写 PR |
| 开发 | 在自己的分支做单一主题的修改，共享文件标明具体函数与影响端 |
| 提交 PR | 说明前后行为、相关检查、部署组合、文档更新和未验证项 |
| 审查合并 | 另一人看差异，共享接口核对发送/接收端，合入 `main` |
| 联调交接 | 记录提交 SHA、各端版本、测试条件、证据和下一步，双方同步 |

GitHub 创建 Issue 时可选择[问题模板](../../.github/ISSUE_TEMPLATE/bug_report.md)或[任务模板](../../.github/ISSUE_TEMPLATE/task.md)。当前两个功能已有固定分支，继续使用各自分支；新任务和独立文档整理从最新 `main` 建分支，例如 `codex/docs-navigation`。

## 已建立的共同起点

仓库：<https://github.com/lingkonglengfeng/2026-Engineering-Training>，为私有仓库。管理员`lingkonglengfeng`，已有写入协作者`yuquan1314`。

| 分支/标签 | 用法 |
| --- | --- |
| `main` | 双方共同合并、联调的版本 |
| `feature/vision-obstacle` | 车主：视觉避障 |
| `feature/turntable-pick-speed` | yuquan1314：原料台抓取提速 |
| `collab-v195-2026-10-27` | 此次同步时的共同基线标签，便于比较/另建回退分支 |
| `codex/freertos_任务规划` | 保留原开发历史，之后使用上述分工分支 |

同步前远端工作分支为V172，远端`main`更早。本次把当前V195源码、电脑视觉、双路桥接、相关规则与回归一起保存，避免一人从V172、一人从V195开工。V195含5号电机原生速度位置模式，3200脉冲/圈尚待实机核对；**共同基线不等于已经完成整车实测**。

## 怎么分工

这里“转盘夹取提速”指原料台旋转物料的抓取周期优化，不是仅提高车载载物盘舵机转速。提速目标应同时记录耗时和抓取成功次数，避免仅调整速度数字。

| 范围 | 主要负责人 | 主要文件 |
| --- | --- | --- |
| 前方圆柱检测、距离和横向估计、无线回传、障碍显示 | 车主 | `vision/pc_vision/obstacle_detection.py`、`obstacle_link.py`、`serial_monitor.py`、CAM2桥接 |
| 主控地图、障碍坐标落图和绕行 | 车主 | `Core/Src/vision_obstacle.c`、`grid_nav.c`及对应头文件/测试 |
| 原料台抓取时机、夹爪/升降/伸缩协同、取放周期 | yuquan1314 | `Core/Src/arm_material.c`、`arm_round1.c`、`arm_start_pose.c`及对应头文件/测试 |
| CAM1抓取视觉、圆周预测、顶面定位 | yuquan1314按提速需要修改 | `vision/pc_vision/backend.py`、`motion_prediction.py`、`material_surface.py`、`cylinder_top.py`、`color_classifier.py`；修改结果回传含义时双方审查 |
| 公共驱动和升降标定、夹爪/载物盘公共参数 | 双方共同审查 | `arm_driver.*`、`arm_calibration.*`、`arm_servo.*`；V195的5号原生定位属于共同基线 |
| 大文件中的总流程/导航接入、通信与界面 | 双方共同审查 | `zdt_motor.c`、`match_control.*`、`vision.*`、共享`bridge.py`、`gui.py`、`settings.py`、`.ioc`、`.uvprojx` |

`zdt_motor.c`不能简单划给某一人：车主主要改`ZDT_Vobs_*`及其导航接入；队友若需要修改任务调度/抓料接入，先在PR中标出具体函数。不要为了各自功能在同一个PR里重排或格式化整个文件。双方尽量先在独立模块实现，再小范围接入共享文件。

协议保持现有8字节语义；新增障碍专用0x86不挪用旧0x08。接口、单位、颜色编号、坐标方向或主控状态机语义改变时，先说明发送端/接收端如何一起更新。新增或改变停机保护前先告知车主。

## 队友第一次下载

在自己的电脑打开PowerShell/Git终端，选一个工程父目录：

```powershell
git clone https://github.com/lingkonglengfeng/2026-Engineering-Training.git
cd 2026-Engineering-Training
git switch --track origin/feature/turntable-pick-speed
git status
```

HTTPS登录请使用Git Credential Manager/浏览器登录；GitHub不再接受账号密码作为Git推送密码。已配SSH也可使用仓库SSH地址。不要复制别人电脑的登录令牌。

若已有旧工程，先提交自己需要的修改到自己的分支，再同步；不确定时把新版本clone到另一目录，保留旧目录对照。不要对有未保存修改的目录直接使用`reset --hard`。

车主本地当前工程已经切到`feature/vision-obstacle`，无需重新clone。另一台新电脑使用同样clone步骤，最后改为`git switch --track origin/feature/vision-obstacle`。

### 使用 GitHub Desktop

车主选择 File → Add local repository，添加当前 `E:\Engineering_Training_Project`；队友选择 File → Clone repository，下载上述私有仓库。在顶部 Current branch 中选择各自工作分支，Fetch origin 刷新远端；收到 Pull origin 提示时同步自己的分支。需要吸收另一人的已合并成果时，先提交当前修改，切到 `main` 并 Fetch/Pull 到最新，再切回自己的分支，使用 Branch → Merge into current branch 选择 `main`。不要把另一人的代码提交在自己的本地 `main` 上。

Changes 中只勾选本次要提交的文件，填写 Summary 后 Commit，再 Push origin。Create Pull Request 打开 GitHub 后核对 base=`main`、compare=自己的功能分支。GitHub Desktop 的提交、推送和合并是三个不同操作。

若提示认证失败，先确认登录了有仓库权限的账号，并检查 Repository settings → Remote 中的仓库地址。本机当前使用 HTTPS；网络连接通过本机已开启的代理，Git 的仓库级代理设置不会随源码克隆到队友电脑。队友应按自己网络配置，不能照抄本机代理端口。

## 每次开始工作

先看`git status`，有改动就先提交。以队友为例：

```powershell
git switch feature/turntable-pick-speed
git fetch origin
git pull --ff-only
git merge origin/main
```

车主把分支名换成`feature/vision-obstacle`。`git pull --ff-only`同步自己的远端分支；`git merge origin/main`吸收已合入的另一方成果。没有新提交时显示Already up to date即可。选择merge便于两人初期保留历史，不需要强推。

## 提交和推送

以一次原料台抓取优化为例，只添加本次实际修改的文件：

```powershell
git diff
git add Core/Src/arm_material.c
git add tools/tests/run_arm_material_tests.py
git diff --cached
git commit -m "perf(arm): reduce turntable pickup cycle"
git push
```

车主可用`feat(vision): ...`、`fix(obstacle): ...`等明确主题。一条提交处理一项可说明的改动，功能与无关格式化分开。推送成功后对方能fetch到你的分支，但`main`和对方工作目录都不会自动合并。

## 用Pull Request合并

1. 在GitHub打开仓库 → Pull requests → New pull request。
2. base选`main`，compare选自己的功能分支。**两条刚创建且相同的分支没有差异，等实际提交后再开PR。**
3. 写清解决什么问题、共享文件变化和实际验证。未完成可创建Draft PR供另一人先看。
4. 请另一人Review。核对文件差异；涉及协议/共享驱动时同时看两端调用。CODEOWNERS会提示相应审查人，它本身不强制阻止直接push。
5. 有问题就在原分支继续修改、提交、push，同一个PR会更新。准备好后在GitHub选择Create a merge commit合入`main`，保留本次初始两条工作分支。
6. 另一人按上面的“每次开始工作”先合并最新`origin/main`，再继续。先合哪一个PR都可以，后合者负责解决冲突并重新验证。

这两条分支只用于当前两个任务，不作为永久分工。任务完成后，后续功能从更新后的`main`新建命名明确的分支，例如`git switch -c feature/obstacle-distance-calibration origin/main`。等当前两项都结束且双方已同步，再删除旧功能分支。

## 冲突怎么处理

```powershell
git fetch origin
git merge origin/main
git status
```

Git提示冲突时，打开列出的文件，逐段处理`<<<<<<<`、`=======`、`>>>>>>>`，保留双方需要的行为；不能整文件“全部用我的”或“全部用他的”。处理后`git add <已处理的文件>`，重新编译并跑相关回归，最后`git commit`、`git push`。暂时不想继续可用`git merge --abort`取消这次合并；它不会替你保存合并前未提交的工作，所以合并前先提交。

出现“提交被拒绝/non-fast-forward”先fetch并检查远端提交，不用`git push --force`。共享分支不重写已推送历史。错误已合入时通过Revert PR/`git revert`新增撤销提交，保留对方历史。

## 本项目验证与本地文件

只改说明/协作配置时检查链接、路径和 `git diff --check`，不要求跑整车。改固件至少用 Keil 构建并记录错误和警告；有硬件动作变化时附相同条件的耗时及成功次数。离线测试不等于实车通过。

工具链、依赖安装、按模块检查命令与基线历史失败统一维护在[验证与联调](验证与联调.md)，避免这里与首页分别保存一套长清单。按本次影响范围选择检查，未执行和未实测分别标注；不要为了通过旧参数预期而修改当前配置。

以下保留在自己电脑：`output/`、编译HEX/AXF/MAP、`tmp/`、仿真输出、截图/日志、`config.json`、`config.cam2.json`、`serial_config.json`、现场`calibration.json`、Keil个人`.uvoptx`。示例配置、代码、必要测试样例和部署所需相机应用保存在仓库；已有历史产物未重写历史清除。需要分享固件/日志时用GitHub Release或单独文件，不把整个output目录提交。

## 联调与交接

合并源码与更新设备分开记录。合并后双方先同步 `main`，再按 PR 说明更新需要变更的端；不要只更新 PC 而遗漏必须配套的相机桥或主控。

交接直接写进当前 PR / Issue，复制[交接记录模板](验证与联调.md#交接记录模板)。至少给出提交 SHA、固件/PC/两相机版本组合、执行的检查、实车结果或“未实测”、待确认项和回退入口。抓取提速记录同条件耗时与成功/总次数，避障记录净距、车速、障碍位置与绕行结果。

操作步骤见[现场联调顺序](验证与联调.md#现场联调顺序)。正常避障暂停后重规划；`NO_PATH` 保持暂停并打印原因，不新增 `MATCH FAULT` 或伪报完成工位。手动清障和 `vobs retry` 只用于非正式调试或赛前/赛后；正式比赛限制见 [V194 说明](../electrical/2026-10-27_CAM2障碍融入比赛流程_V194.md#无路及比赛规则)。

## 官方教程

- [GitHub Flow：分支、提交、PR、审查和合并](https://docs.github.com/en/get-started/using-github/github-flow)
- [创建Pull Request](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request)
- [解决合并冲突](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/addressing-merge-conflicts/resolving-a-merge-conflict-using-the-command-line)
- [CODEOWNERS审查人规则](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/about-code-owners)
