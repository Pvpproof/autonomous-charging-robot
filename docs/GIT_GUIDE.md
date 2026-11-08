# 日常使用 Git

Git 提交是文件版本快照。它不会自动把文件传到 GitHub；推送是另一个步骤。

## 查看

在此仓库目录运行：

```bash
git status
git diff
git log --oneline --decorate -10
```

依次查看修改状态、尚未暂存的差异和已有提交。

## 保存一个完成的修改

```bash
git add README.md
git diff --cached
git commit -m "docs: explain current development status"
```

把示例文件名换成此次明确需要保存的文件。提交前逐行检查暂存内容，避免直接添加整个目录。忽略规则不会阻止已经被追踪的敏感文件进入后续提交。

新功能开始前，先确认当前修改属于谁，再建立分支：

```bash
git switch -c feature/recorded-route-return
```

这会切换工作区；另一线程正在使用同一目录时，应改用独立克隆或工作树，不能并发切换分支。

## 查看与撤销旧版本

```bash
git show <提交号>:<文件路径>
```

这只显示旧文件，不覆盖当前文件。

需要撤销已提交修改时，先保存当前工作，再评估执行 `git revert <提交号>`。它新增一次反向提交，保留原历史；遇到冲突需要人工处理。不要在别的开发线程工作期间执行 `reset --hard`、`clean` 或覆盖工作区。

回退文件不会撤销已经发生的车辆运动，也不应恢复或删除设备运行锁来代替现场确认。

## GitHub

仅在远程仓库地址已经核实、发布文件经过检查后配置 origin 和推送。不要把密钥、地图、日志或厂家配置强制加入 Git，也不要覆盖远程历史。
