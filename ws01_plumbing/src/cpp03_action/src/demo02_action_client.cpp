/* 
 需求：
 步骤：
   1.包含头文件；
   2.初始化 ROS2 客户端；
   3.定义节点类；
     3-1.创建发布方；
     3-2.创建定时器；控制发布频率
     3-3.组织消息并发布。
   4.调用spin函数，并传入节点对象指针；
   5.释放资源.

 */


#include "rclcpp/rclcpp.hpp"




class ProgressActionClient : public rclcpp::Node{ 
public:

    ProgressActionClient():Node("progress_action_client_node_cpp"){  
      RCLCPP_INFO(this->get_logger(), "action client creation!");
    }

};

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProgressActionClient>());
  rclcpp::shutdown();

  return 0;
}
