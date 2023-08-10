/* 
 需求：编写动作服务端，需要解析客户端提交的数字，遍历该数字并累加求和，最终结果响应回客户端，
      且请请求响应过程中需要生成连续反馈。
 分析：
      1.创建动作服务端；
      2.处理请求数据；
      3.处理取消任务请求；
      4.生成连续反馈。
      5. 响应族中结果
      
 步骤：
   1.包含头文件；
   2.初始化 ROS2 客户端；
   3.定义节点类；
      3-1.创建动作服务端；
      3-2.处理请求数据；
      3-3.处理取消任务请求；
      3-4.生成连续反馈。
   4.调用spin函数，并传入节点对象指针；
   5.释放资源.

 */


#include "rclcpp/rclcpp.hpp"




class ProgressActionServer : public rclcpp::Node{ 
public:

    ProgressActionServer():Node("progress_action_server_node_cpp"){  
      RCLCPP_INFO(this->get_logger(), "action server creation!");
    }

};

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProgressActionServer>());
  rclcpp::shutdown();

  return 0;
}
