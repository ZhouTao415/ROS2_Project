# 1. helloworld.cpp
  流程；
     1.包含头文件；
     2.初始化ROS2客户端；
     3.创建结点指针
     4.输出日志
     5.释放资源
  
   
   ## Method 1, not recommend
   ```bash
   #include "rclcpp/rclcpp.hpp"
   
    int main(int argc, char **argv){
    
    //  2.初始化ROS2客户端
    rclcpp::init(argc, argv);
    
    //  3.创建结点指针
    //rclcpp 有个Node类，Node类有个函数，之后给u节点起个名字    
    auto node = rclcpp::Node::make_shared("helloworld_node_cpp");
    
    //  4.输出日志
    RCLCPP_INFO(node->get_logger(), "hello world!");
    
    //  5.释放资源
    rclcpp::shutdown();

    return 0;
}
   ```
   ##  Method2： recommend
      之所以继承比直接实例化Node更被推荐，是因为继承方式可以在一个进程内组织多个节点，
      这对于提高节点间的通信效率是很有帮助的，但是直接实例化则与该功能不兼容
      
  ```bash
// 自定义类继承 Node
class MyNode: public rclcpp::Node {
public: 
        MyNode():Node("hello_node_cpp"){ //构造函数， 设置名称
            RCLCPP_INFO(this->get_logger(), "hello world!(继承方式)");
        }
};

int main(int argc, char const *argv[])
{
    // initialization
    rclcpp::init(argc, argv);

    // 实例化自定义类
    auto node = std::make_shared<MyNode>();
    
    //  资源释放
    rclcpp::shutdown();
    
    return 0;
}
```

