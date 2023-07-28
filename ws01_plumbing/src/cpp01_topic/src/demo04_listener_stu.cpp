/*  TalkerStu
    需求：订阅发布方发布的学生消息，并输出到终端。
    步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
        3-1.创建订阅方；
        3-2.解析并处理订阅到的消息。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;

class ListenerStu : public rclcpp::Node{
public:
  ListenerStu():Node("listenerstu_node_cpp"){
    RCLCPP_INFO(this->get_logger(), "订阅方创建!");
    subscroption_ = this->create_subscription<Student>("chatter_stu", 10, std::bind(&ListenerStu::do_callback, this, std::placeholders::_1));
  }

private:

  void do_callback(const Student &stu) {
    RCLCPP_INFO(this->get_logger(), "订阅的学生信息： name = %s, age = %d, height = %.2f", stu.name.c_str(), stu.age, stu.height);
  }
  rclcpp::Subscription<Student>::SharedPtr subscroption_;

};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ListenerStu>());
  rclcpp::shutdown();
}

// // 1.包含头文件；
// #include "rclcpp/rclcpp.hpp"
// #include "base_interfaces_demo/msg/student.hpp"
 
// using std::placeholders::_1;
// using base_interfaces_demo::msg::Student;
// // 3.定义节点类；
// class ListenerStu : public rclcpp::Node
// {
//   public:
//     ListenerStu()
//     : Node("student_subscriber")
//     {
//       // 3-1.创建订阅方；回调函数是有参数的，参数就是订阅到的Student的消息，需要用占为符号。
//       subscription_ = this->create_subscription<Student>("topic_stu", 10, std::bind(&ListenerStu::topic_callback, this, std::placeholders::_1));
//     }

//   private:
//     // 3-2.处理订阅到的消息； & msg这里是引用
//     void topic_callback(const Student & msg) const
//     {
//       RCLCPP_INFO(this->get_logger(), "订阅的学生消息：name=%s,age=%d,height=%.2f", msg.name.c_str(),msg.age, msg.height);
//     }
//     rclcpp::Subscription<Student>::SharedPtr subscription_;
// };

// int main(int argc, char * argv[])
// {
//   // 2.初始化 ROS2 客户端；
//   rclcpp::init(argc, argv);
//   // 4.调用spin函数，并传入节点对象指针。
//   rclcpp::spin(std::make_shared<ListenerStu>());
//   // 5.释放资源；
//   rclcpp::shutdown();
//   return 0;
// }