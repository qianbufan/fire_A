# fire_A
灭火仿真1分钟3面
## 安装依赖
Ubuntu22.04 ROS2 Humble MoveIt2

## 功能包简介
arm_moveit_config：主程序运行功能包，由MoveIt2 assistant生成  
robot：机器人模型和基础配置    
pymoveit2：moveit2对python接口，需先安装

## 使用方法
1. 加载环境变量  
cd 工作空间 #进入  
colcon build #构建
source install/setup.bash #加载环境变量  
chmod +x src/arm_moveit_config/scripts/*.py #给python脚本执行权限  
2. 启动机器人  
ros2 launch arm_moveit_config demo.launch.py   
#MoveIt规划器启动较慢，等终端出现You can start planning now!后再重新加载Rviz中的MoveIt规划器  
3. 加载世界物品  
在另一个终端中：python src/arm_moveit_config/scripts/world_marker.py  
#Rviz中添加marker并选择topic  
4. 开始移动  
在另一个终端中：python src/arm_moveit_config/scripts/move.py  
#可以在Rviz中看到机器人的运动  
<img width="1751" height="1235" alt="image" src="https://github.com/user-attachments/assets/980a6065-8c2d-4dda-afef-6053b62a7def" />
