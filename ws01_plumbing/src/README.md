# ROS2_Project

## 1. 话题通信之原生消息

### Publisher: 话题发布 
```bash
ros2 run cpp01_topic demo01_talker_str 
```
### Subscription: 话题订阅
```bash
ros2 run cpp01_topic demo02_listener_str 
```
## 2. 话题通信之自定义接口消息

### Publisher: 话题发布 
```bash
ros2 run cpp01_topic demo03_talker_stu
```

### 检验话题是否发布出去 

```bash
ros2 topic echo /chatter_stu
```
"chatter_stu" is topic name 
