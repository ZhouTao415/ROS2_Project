/*
   需求：处理Client请求发送的目标点，,获取原生乌龟的坐标，计算乌龟与目标点之间的直线距离，响应给Client。
   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.创建乌龟姿态订阅方，回调函数中获取x坐标与y坐标；(原生乌龟pose /turtle/pose)
            3-2.创建服务端；
            3-3.解析目标值，计算距离并反馈结果。(回调函数需要解析客户端数据并响应结果到客户端)
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。

    //    ros2 service call /distance base_interfaces_demo/srv/Distance "{'x': 8.54, 'y': 9.54, 'theta': 0.0}"
*/


#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using base_interfaces_demo::srv::Distance;

class Exer02Server : public rclcpp::Node{ 
public:

    Exer02Server() : Node("exer02_server_node_cpp"), turtle1_x(0.0), turtle1_y(0.0) {  
        RCLCPP_INFO(this->get_logger(), "案例2Server创建!");
        // 3-1.创建乌龟姿态订阅方，回调函数中获取x坐标与y坐标；(原生乌龟pose /turtle1/pose)
        sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&Exer02Server::poseCallBack, this, _1));
        // 3-2.创建服务端；
        server_ = this->create_service<Distance>("distance", std::bind(&Exer02Server::distanceCallBack, this, _1, _2));
        // 3-3.解析目标值，计算距离并反馈结果。(回调函数需要解析客户端数据并响应结果到客户端)
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Service<Distance>::SharedPtr server_;
    float turtle1_x, turtle1_y;
    void poseCallBack(const turtlesim::msg::Pose::SharedPtr pose) {
        // x,y 的目的是为了接收变量
        turtle1_x = pose->x;
        turtle1_y = pose->y;
    }

    void distanceCallBack(const Distance::Request::SharedPtr request, Distance::Response::SharedPtr response) {
        // 解析目标值
        float goal_x = request->x;
        float goal_y = request->y;
        // 距离计算
        float x = goal_x - turtle1_x;
        float y = goal_y - turtle1_y;

        float distance = std::sqrt(x * x + y * y);
        
        // 将结果设置到响应客户端
        response->distance = distance;
        RCLCPP_INFO(this->get_logger(),"目标坐标:(%.2f,%.2f),原生乌龟坐标:(%.2f,%.2f), 距离:%.2f",goal_x,goal_y,turtle1_x, turtle1_y,response->distance);

    }
};

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Exer02Server>());
  rclcpp::shutdown();

  return 0;
}