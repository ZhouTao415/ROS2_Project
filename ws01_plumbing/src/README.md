# ROS2_Project

## 话题通信

### 1. 话题通信之原生消息

#### Publisher: 话题发布 
```bash
ros2 run cpp01_topic demo01_talker_str 
```
#### Subscription: 话题订阅
```bash
ros2 run cpp01_topic demo02_listener_str 
```
### 2. 话题通信之自定义接口消息

#### Publisher: 话题发布 
```bash
ros2 run cpp01_topic demo03_talker_stu
```

## 服务通信
终端下进入工作空间的src目录，创建C++功能包
```bash
ros2 pkg create cpp02_service --build-type ament_cmake --dependencies rclcpp base_interfaces_demo --node-name demo01_server
```
pkg: cpp02_service 功能包名,
build-type: ament_cmake C++ 类型,
dependencies:  rclcpp 和 base_interfaces_demo 依赖包,
node-name: demo01_server 节点名称

## 常见问题

### 1. 检验话题是否发布出去 
"chatter_stu" is topic name 

```bash
ros2 topic echo /chatter_stu
```

#### 2. vscode的Terminal运行而Ubuntu自带的Terminal不运行的原因
因为没有在Ubuntu自带的Terminal中

```bash
colcon build
```



