# PX4 仿真退出与进程清理

## 正常结束

在运行 bash Simulation/start.sh 的终端按一次 Ctrl+C。入口转发信号后等待实际仿真程序退出，仿真程序先清理本次 PX4、RViz、ROS 节点及 ROS 启动器，再保存轨迹和报告。看到 Process cleanup: complete，并等待终端提示符返回，才算退出完成。

清理不是瞬时的；伪终端验证约 11.85 s 完成进程清理，报告随后保存。清理期间重复 Ctrl+C 不会中止 finally；关闭整个终端或 kill -9 不等同于正常 Ctrl+C 退出。

Ctrl+C 取消正在执行的任务时，result.json 记录 failed / Run interrupted，终端可能显示 SITL validation failed。这表示任务被取消，不表示进程清理失败。清理状态单独保存在 cleanup.json 和 result.json.cleanup。

--keep-open 仅在任务成功后暂停物理并保留窗口，Ctrl+C 仍然执行退出清理。

## 清理已脱离入口的旧进程

已在运行的旧脚本不会因代码更新自动获得修复。需要按它对应的输出目录清理：

```bash
python3 Simulation/run.py px4 stop --output "$RUN" --dry-run
python3 Simulation/run.py px4 stop --output "$RUN"
```

这里 RUN 必须是那次仿真的输出目录，包含 manifest.json。--dry-run 只显示归属进程，不发送信号。工具按精确 ROS_HOME=<该目录>/ros_home 匹配本用户进程，逐次核对 PID 启动时间；不按 px4、rviz 或 ROS 节点名称全局终止进程。

停止成功输出 status=complete、remaining=[]；否则返回非零并在 cleanup.json 留下未退出进程。不要通过删除 /tmp/px4_lock-* 代替退出正在运行的 PX4；锁文件存在本身不等于实例被占用。

## 修复方式

- 两层 Python 入口使用 exec 转交命令，不再因 subprocess.call 的 KeyboardInterrupt 等待很短而丢下真实运行器。
- Bash 为运行器创建独立会话，转发 INT/TERM，并在 wait 被信号中断后继续等待清理。运行器显式恢复 SIGINT 处理，避免 Bash 异步启动继承忽略信号。
- 清理依据本次 ROS_HOME，包含另建进程组/会话的 ROS 后代，即使 roslaunch 已退出也能找到继承标记的节点。
- 导航退出期间保留 Gazebo 时钟与 ROS master，随后清理它们；需要时逐级使用 SIGINT、SIGTERM、SIGKILL。
- processes.json 记录控制器和直接启动进程；cleanup.json 记录信号、耗时和剩余进程。

## 验证

- 97 项测试通过：69 项 PX4 测试、28 项 forest_lio 测试。新增四项覆盖脱离父进程的独立会话孙进程、强制升级、时钟退出顺序、PID 启动标识不匹配、重复中断，以及其他输出目录的进程仍存活。
- 真实伪终端启动完整森林入口，SITL 10 s、OFFBOARD 解锁时确认 PX4 / RViz / fastlio_mapping 同时运行，再注入 Ctrl+C。清理后归属进程为空，报告存在，另一个输出目录的哨兵仍存活。
- 这里只验证退出链路，不代表完成 600 s 路线。

证据：
- [真实终端日志](../../../artifacts/px4_forest/ctrl_c_forest_20261019.console.log)
- [清理记录](../../../artifacts/px4_forest/ctrl_c_forest_20261019/cleanup.json)
- [退出验证](../../../artifacts/px4_forest/ctrl_c_forest_20261019/interrupt_validation.json)
- [取消后的报告](../../../artifacts/px4_forest/ctrl_c_forest_20261019/report.md)


