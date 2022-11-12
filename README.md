# iiwa_ros

### gazebo仿真时使用如下命令：

```sh
1: roslaunch iiwa_moveit moveit_planning_execution.launch
```
```sh
2: roslaunch iiwa_description iiwa7_sin.launch
```

### Rviz+MoveIt时使用如下命令：

```sh
1: roslaunch iiwa_moveit demo.launch 
```
```sh
2: rosrun iiwa_description markerpub_sin.py  # 显示轨迹
```
```sh
3: rosrun iiwa_description moveit_sin.py  # 画图
```
