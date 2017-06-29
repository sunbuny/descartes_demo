## 华东师范大学-机器人学课程大作业
## 安装
git clone 该项目后, sh setup.sh自动构建项目环境
## 运行
命令行输入
```
roslaunch draw demo_setup.launch  // 运行irb2400的moveit仿真环境
任务一 控制机器人画一个圈
roslaunch draw demo_run.launch    // 控制机器人画一个圆
任务二 使用键盘控制机器人
roslaunch draw demo_key_ctrl.launch  //使用键盘控制机器人
-------use W A S D to control-----------------------
-------W move alone world x axis 20 cm--------------------
-------S move alone world x axis-20 cm--------------------
-------A move alone world y axis-20 cm--------------------
-------D move alone world y axis 20 cm--------------------
```
