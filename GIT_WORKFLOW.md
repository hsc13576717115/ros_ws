# Git 工作流说明

这份文档针对当前工作区 `/home/orangepi/ros_ws`，目的是让你可以直接按步骤完成：

- 日常调参后上传到 GitHub
- 新功能开发后合并回主分支
- 出问题时快速回溯历史版本
- 保存一个“当前稳定可用”的节点

当前仓库默认分支是 `main`，远端已经配置好，可以直接在工作区根目录执行 Git 命令。

## 1. 基本原则

始终先进入工作区根目录：

```bash
cd /home/orangepi/ros_ws
```

建议遵守这几个原则：

- 小改动频繁提交，方便回溯
- 调参和新功能尽量不要混在同一个 commit
- 做新功能时尽量开分支，不要直接在 `main` 上长期开发
- 只在确认能跑后再合并到 `main`
- 回退历史优先用 `git revert`，不要随便用破坏性的 `reset --hard`

## 2. 先看当前状态

每次改代码或改参数前后，先执行：

```bash
cd /home/orangepi/ros_ws
git status
```

常见输出含义：

- `modified:` 文件被修改了但还没加入暂存区
- `new file:` 新增文件
- `deleted:` 文件被删除
- `Changes to be committed:` 已加入暂存区，下一次 commit 会提交
- `nothing to commit, working tree clean` 工作区干净

## 3. 日常调参的最简单流程

适用于：

- 改 `nav2_slam_params.yaml`
- 改 launch 参数
- 改控制桥接脚本
- 小范围修 bug

建议流程：

```bash
cd /home/orangepi/ros_ws
git status
git add src/nav2_config/config/nav2_slam_params.yaml
git add src/nav2_config/launch/slam_navigation.py
git commit -m "nav2: tune near-goal behavior"
git push
```

如果这次改动不止两个文件，也可以：

```bash
git add .
git commit -m "nav2: tune goal approach"
git push
```

但更推荐按文件精确 `git add`，避免把不想提交的临时改动一起传上去。

## 4. 推荐的 commit 写法

提交信息尽量做到“看名字就知道改了什么”。

推荐格式：

```bash
模块: 动作
```

例如：

```bash
git commit -m "nav2: tune goal tolerances"
git commit -m "nav2: switch replanning BT to distance trigger"
git commit -m "controller: reduce angular smoothing"
git commit -m "docs: update workspace README"
git commit -m "slam: adjust cartographer imu config"
```

不推荐这种太模糊的写法：

```bash
git commit -m "update"
git commit -m "fix"
git commit -m "test"
```

## 5. 新功能开发建议走分支

适用于：

- 增加新导航策略
- 新增一个 launch
- 接入新传感器
- 新增感知节点
- 对底层控制链路做结构性修改

### 5.1 从 `main` 拉一个新分支

先确保主分支最新：

```bash
cd /home/orangepi/ros_ws
git switch main
git pull --ff-only
```

创建并切换到新分支：

```bash
git switch -c feature/nav2-precise-parking
```

分支命名建议：

- `feature/xxx` 新功能
- `fix/xxx` 修 bug
- `tune/xxx` 调参优化
- `docs/xxx` 文档修改

### 5.2 在分支上开发并提交

```bash
git status
git add src/nav2_config
git commit -m "nav2: add precise parking behavior"
git push -u origin feature/nav2-precise-parking
```

第一次推送分支建议带 `-u`，后面直接 `git push` 即可。

### 5.3 分支开发完成后合并回 `main`

先回主分支并更新：

```bash
cd /home/orangepi/ros_ws
git switch main
git pull --ff-only
```

合并开发分支：

```bash
git merge --no-ff feature/nav2-precise-parking
git push
```

这里用 `--no-ff` 的原因是：即使分支历史可以快进，Git 也会保留一个明确的合并节点，后面回溯“这个功能是哪次并进来的”会更清楚。

开发分支不需要保留时可以删除：

```bash
git branch -d feature/nav2-precise-parking
git push origin --delete feature/nav2-precise-parking
```

## 6. 每次开始工作前怎么同步远端

如果你换了一台机器，或者 GitHub 上已有新提交，先同步：

```bash
cd /home/orangepi/ros_ws
git switch main
git pull --ff-only
```

为什么推荐 `--ff-only`：

- 如果本地没有分叉，它会安全地拉最新代码
- 如果出现分叉，它会直接报错，不会悄悄给你制造一个你没注意到的 merge commit

## 7. 本地改了一半，临时不想提交怎么办

用 `stash` 临时保存：

```bash
cd /home/orangepi/ros_ws
git stash push -m "wip nav2 tuning"
```

查看暂存列表：

```bash
git stash list
```

恢复最近一次暂存：

```bash
git stash pop
```

如果你只是想临时切分支、拉代码，`stash` 很实用。

## 8. 冲突怎么处理

冲突通常发生在：

- 你和远端都改了同一个文件
- 你合并一个功能分支回 `main`

如果 `git pull` 或 `git merge` 提示冲突：

```bash
git status
```

然后打开冲突文件，你会看到类似内容：

```text
<<<<<<< HEAD
你的内容
=======
对方内容
>>>>>>> branch-name
```

处理方式：

1. 手动改成你真正想保留的最终内容
2. 删除 `<<<<<<<`、`=======`、`>>>>>>>` 这些标记
3. 保存文件
4. 标记冲突已解决

```bash
git add 冲突文件
```

如果是 merge 冲突，继续完成提交：

```bash
git commit
```

如果是 `pull --ff-only` 失败，通常先不要强拉。先看本地和远端分别改了什么，再决定是 rebase 还是 merge。你也可以先找我帮你处理。

## 9. 如何看历史，方便回溯

看最近的提交：

```bash
cd /home/orangepi/ros_ws
git log --oneline --graph --decorate -20
```

看某次提交改了什么：

```bash
git show 提交号
```

例如：

```bash
git show 1d246d6
```

看两个版本之间差异：

```bash
git diff 旧提交号 新提交号
```

看某个文件是怎么一步步被改的：

```bash
git log --oneline -- src/nav2_config/config/nav2_slam_params.yaml
```

## 10. 回退到以前的版本

### 10.1 推荐做法：撤销某次提交，但保留历史

如果某个 commit 改坏了，最安全的是：

```bash
cd /home/orangepi/ros_ws
git revert 提交号
git push
```

这会生成一个“反向提交”，很适合已经推到 GitHub 的历史。

例如：

```bash
git revert 1d246d6
git push
```

### 10.2 恢复某个文件到旧版本

如果只想恢复单个文件：

```bash
git restore --source 提交号 -- 路径
```

例如恢复旧版 Nav2 参数：

```bash
git restore --source 0969f5e -- src/nav2_config/config/nav2_slam_params.yaml
git add src/nav2_config/config/nav2_slam_params.yaml
git commit -m "nav2: restore previous slam params"
git push
```

## 11. 给稳定版本打标签

当你遇到“这一版导航最稳”“这一版能稳定停车”时，建议打标签，后续特别好找。

创建标签：

```bash
cd /home/orangepi/ros_ws
git tag -a stable-nav-2026-03-07 -m "Stable indoor navigation tuning"
git push origin stable-nav-2026-03-07
```

查看所有标签：

```bash
git tag
```

查看某个标签对应内容：

```bash
git show stable-nav-2026-03-07
```

这个功能很适合你做参数阶段性收敛时使用。

## 12. 推荐给你的一套实际工作习惯

如果只是调参数：

```bash
cd /home/orangepi/ros_ws
git switch main
git pull --ff-only

# 修改参数并测试

git add src/nav2_config/config/nav2_slam_params.yaml
git add src/nav2_config/launch/slam_navigation.py
git commit -m "nav2: tune precise parking"
git push
```

如果是加新功能：

```bash
cd /home/orangepi/ros_ws
git switch main
git pull --ff-only
git switch -c feature/new-docking-flow

# 开发并测试

git add .
git commit -m "nav2: add docking flow"
git push -u origin feature/new-docking-flow

git switch main
git pull --ff-only
git merge --no-ff feature/new-docking-flow
git push
```

如果想保留一个稳定节点：

```bash
git tag -a stable-名字-日期 -m "说明"
git push origin stable-名字-日期
```

## 13. 常用命令速查

```bash
cd /home/orangepi/ros_ws
git status
git add 文件名
git add .
git commit -m "说明"
git push
git pull --ff-only
git switch main
git switch -c 新分支名
git branch
git log --oneline --graph --decorate -20
git show 提交号
git revert 提交号
git stash push -m "临时保存"
git stash pop
git tag
```

## 14. 这个仓库里你最常用的几个场景

### 场景 1：Nav2 参数刚调好，准备保存

```bash
git add src/nav2_config/config/nav2_slam_params.yaml
git commit -m "nav2: tune goal approach thresholds"
git push
```

### 场景 2：改了 launch 和桥接脚本，准备一起保存

```bash
git add src/nav2_config/launch/slam_navigation.py
git add src/nav2_config/scripts/cmd_vel_to_move_cmd.py
git commit -m "nav2: refine command smoothing pipeline"
git push
```

### 场景 3：某次实验失败，想回到之前稳定参数

```bash
git log --oneline -- src/nav2_config/config/nav2_slam_params.yaml
git restore --source 某个提交号 -- src/nav2_config/config/nav2_slam_params.yaml
git add src/nav2_config/config/nav2_slam_params.yaml
git commit -m "nav2: restore stable parameters"
git push
```

### 场景 4：做一个较大的新功能，不想污染主线

```bash
git switch main
git pull --ff-only
git switch -c feature/你的功能名
```

## 15. 最后一个建议

对你这种经常调参数、做实验、还需要快速回溯的场景，最实用的不是复杂 Git 技巧，而是这三件事：

1. 每次实验结束立刻 commit
2. 稳定版本立刻打 tag
3. 大功能单独开 branch

这样后面要找“哪一版停车最准”“哪一版不会绕圈”，速度会快很多。
