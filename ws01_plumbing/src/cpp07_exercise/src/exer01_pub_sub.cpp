/*
   需求：订阅窗口1中的乌龟的位姿信息，解析线速度和角速度，然后生成控制窗口2乌龟运动的指令并发布。
   明确：
         订阅话题： //ros2 topic list
                  /turtle1/pose 

         订阅消息： // ros2 topic type /turtle1/pose 
                  turtlesim/msg/Pose

                  // ros2 interface proto turtlesim/msg/Pose
                  "x: 0.0
                  y: 0.0
                  theta: 0.0
                  linear_velocity: 0.0
                  angular_velocity: 0.0
                  "

         发布话题：/t2/turtle1/cmd_vel
         发布消息： // ros2 topic type  /t2/turtle1/cmd_vel
                  geometry_msgs/msg/Twist

                  // ros2 interface proto geometry_msgs/msg/Twist
                  "linear:
                    x: 0.0
                    y: 0.0
                    z: 0.0
                  angular:
                    x: 0.0 --翻滚
                    y: 0.0 --俯仰
                    z: 0.0 --左右转
                  "

   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
          3-1.创建控制第二个窗体乌龟运动的发布方；
          3-2.创建订阅第一个窗体乌龟pose的订阅方；
          3-3.根据订阅的乌龟的速度生成控制窗口2乌龟运动的速度消息并发布。
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。

       bug 描述： 乌龟1后退时候，乌龟2仍然前进
       BUG 原因： 
            1. 和乌龟pose发布有关，当乌龟实际速度为负数时候，pose的速度仍是正数
            2. 发布的乌龟2的速度，与pose中的线速度一致
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

class Exer01PubSub : public rclcpp::Node{ 
public:

    Exer01PubSub():Node("exer_01_pub_sub_node_cpp") {  
      RCLCPP_INFO(this->get_logger(), "案例1对象创建!");
      // 3-1.创建控制第二个窗体乌龟运动的发布方；
      pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/t2/turtle1/cmd_vel", 10);
      // 3-2.创建订阅第一个窗体乌龟pose的订阅方；
      sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&Exer01PubSub::pose_cb, this, std::placeholders::_1));
    }

private:

    void pose_cb(const turtlesim::msg::Pose &pose) {
      // 3-3.根据订阅的乌龟的速度生成控制窗口2乌龟运动的速度消息并发布。
      // 创建速度指令
      geometry_msgs::msg::Twist twist;
      twist.linear.x = pose.linear_velocity;
      twist.angular.z = -pose.angular_velocity;
      // 发布
      pub_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    size_t count_; 
};

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Exer01PubSub>());
  rclcpp::shutdown();

  return 0;
}