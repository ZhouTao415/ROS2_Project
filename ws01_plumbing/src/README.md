# ROS2_Project

## ● 话题通信 Topic
终端下进入工作空间的src目录，创建C++功能包
```bash
ros2 pkg create cpp01_topic --build-type ament_cmake --dependencies rclcpp std_msgs base_interfaces_demo
```

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
#### Subscription: 话题订阅
```bash
ros2 run cpp01_topic demo04_listener_stu 
```

## ● 服务通信 Service
终端下进入工作空间的src目录，创建C++功能包
```bash
ros2 pkg create cpp02_service --build-type ament_cmake --dependencies rclcpp base_interfaces_demo --node-name demo01_server
```
pkg: cpp02_service 功能包名,
build-type: ament_cmake C++ 类型,
dependencies:  rclcpp 和 base_interfaces_demo 依赖包,
node-name: demo01_server 节点名称

### Server Response: 服务端响应
```bash
ros2 run cpp02_service demo01_server
```
### Client Request: 客户端请求
```bash
ros2 run cpp02_service demo02_client 10 40

```
## ● 动作通信 Action
终端下进入工作空间的src目录，创建C++功能包
```bash
ros2 pkg create cpp03_action --build-type ament_cmake --dependencies rclcpp rclcpp_action base_interfaces_demo --node-name demo01_action_server
```
### Action Server Response: 动作服务端响应
```bash
ros2 run cpp03_action demo01_action_server 
```
### Action Client Request: 动作客户端请求
```bash
ros2 run cpp03_action demo02_action_client 10
```




## 常见问题

### 1. 检验话题是否发布出去 
"chatter_stu" is topic name 

```bash
ros2 topic echo /chatter_stu
```

### 2. vscode的Terminal运行而Ubuntu自带的Terminal不运行的原因
因为没有在Ubuntu自带的Terminal中

```bash
colcon build
```
### 3. 验证Topic/Service/Action接口消息是否正确

```bash
ros2 interface show base_interfaces_demo/srv/AddInts
```
"base_interfaces_demo/srv/AddInts"你定义的接口文件的位置

### 4. 在没有客户端下测试请求响应的数据 
请求我们的客户端 ros2 service call

```bash
ros2 service call /add_ints base_interfaces_demo/srv/AddInts "{'num1': 10, 'num2': 30}"
```

“/add_ints”: 话题名称, 
“base_interfaces_demo/srv/AddInts“: 接口的消息类型, 
“"{'num1': 10, 'num2': 30}"”: json/ymal格式的提交数据

### 5. 检验action是否可以响应 
```bash
ros2 action send_goal /get_sum base_interfaces_demo/action/Progress -f "{'num': 10}"
```
get_sum： 话题名称, 
base_interfaces_demo/action/Progress： 消息类型, 
-f: feedback

