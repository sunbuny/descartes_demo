## 华东师范大学-机器人学课程大作业
## 安装
git clone 该项目后catkin_make即可
## 运行
source 该项目的环境变量之后，运行：
```
roslaunch draw demo_setup.launch  // 运行irb2400的moveit仿真环境
roslaunch draw demo_run.launch    // 控制机器人画一个圆
roslaunch draw demo_key_ctrl.launch  //使用键盘控制机器人
-------use W A S D to control-----------------------
-------W move alone world x axis 20 cm--------------------
-------S move alone world x axis-20 cm--------------------
-------A move alone world y axis-20 cm--------------------
-------D move alone world y axis 20 cm--------------------
```