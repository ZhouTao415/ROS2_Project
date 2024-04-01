/*
   需求：Client发布目标点的坐标，接收并处理服务端反馈的结果。
   步骤：
       0. 解析动态传入的数据， 作为目标点坐标
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.构造函数创建客户端；
            3-2.客户端需要连接服务端连接服务；
            3-3.发送请求数据.
       4.调用对象服务连接、发送请求、处理响应相关函数；
       5.释放资源。
*/


#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

using namespace std::chrono_literals;
using base_interfaces_demo::srv::Distance;

class Exer03Client : public rclcpp::Node{ 
public:

    Exer03Client():Node("exer03_client_node_cpp") {  
        RCLCPP_INFO(this->get_logger(), "案例2Client创建!");
        // 3-1.构造函数创建客户端；
        client_ = this->create_client<Distance>("distance");
    }
    // 3-2.客户端需要连接服务端连接服务；
    bool connect_server() {
      while (!client_->wait_for_service(1s))
      {
        // 迟迟连接不上，不需要等待/* condition */，把客户端程序终止
        if (!rclcpp::ok()){
          // 这里不能用this， 因为节点不ok，关闭了，没有上下文，用rclcpp
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "节点强制退出!");
          return false;
        }
        RCLCPP_INFO(this->get_logger(), "服务连接中...");
      }
      // success connect
      return true;
    }
    // 3-3.发送请求数据.

    // 有一个返回值，类型暂时不明确先用viod
    // void send_goal(float x, float y, float theta) {
    rclcpp::Client<Distance>::FutureAndRequestId send_goal(float x, float y, float theta) {
      auto request = std::make_shared<Distance::Request>();//目前是空的，要设置请求数据
      request->x = x;
      request->y = y;
      request->theta = theta;
      return client_->async_send_request(request);
    }


private:
    rclcpp::Client<Distance>::SharedPtr client_;
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  // 0. 解析动态传入的数据， 作为目标点坐标
  // 6 9 0.0 --ros-args file 5 argument
  // 判断传入的参数个数是否正确
  if (argc != 5)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "请提交x坐标, y坐标与theta三个参数");
    return 1;
  }
  // 解析提交的参数
  float goal_x = atof(argv[1]); //argv[1] is string, change to float, using atof;
  float goal_y = atof(argv[2]); 
  float goal_theta = atof(argv[3]); 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "(x, y, theta): (%.2f, %.2f, %.2f))", goal_x, goal_y, goal_theta);
  
  //  4.调用对象服务连接、发送请求、处理响应相关函数；
    // 节点对象指针创建出来
  auto client = std::make_shared<Exer03Client>();
    // 服务连接
  bool flag = client->connect_server();
  if (!flag)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接失败!");
    return 1;
  }
    // 连接成功，发送请求，处理响应
    // 返回值类型: FutureAndRequestId
  auto future = client->send_goal(goal_x, goal_y, goal_theta);

  //根据返回对象，进行处理，返回有多种可能，判断响应结果状态
  if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(client->get_logger(), "两只乌龟距离: %.2f m", future.get()->distance);
  } else
  {
    RCLCPP_ERROR(client->get_logger(), "服务响应失败!");
  }
  
  rclcpp::shutdown();

  return 0;
}

