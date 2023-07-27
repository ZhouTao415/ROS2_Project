/*  
 需求：以某个固定频率发送文本学生信息，包含学生的姓名、年龄、身高等数据。

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
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;
using namespace std::chrono_literals;

class TalkerStu :public rclcpp::Node{
public:
  TalkerStu() : Node("talkerstu_node_cpp"), age(0){
    publischer_ = this->create_publisher<Student>("chatter_stu", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&TalkerStu::on_timer, this));
  }

private:
  void on_timer(){
    auto stu = Student();
    stu.name = "Xin";
    stu.age = age;
    stu.height = 1.70;
    age++;
    publischer_->publish(stu);
    RCLCPP_INFO(this->get_logger(), "发布的消息：(%s, %d, %2f)", stu.name.c_str(), stu.age, stu.height);
    
  }
  rclcpp::Publisher<Student>::SharedPtr publischer_;
  rclcpp::TimerBase::SharedPtr timer_;
  int age;
};


int main(int argc, char const* argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalkerStu>());
  rclcpp::shutdown();
  
  return 0;
}

// // 1.包含头文件；
// #include "rclcpp/rclcpp.hpp"
// #include "base_interfaces_demo/msg/student.hpp"

// using namespace std::chrono_literals;
// using base_interfaces_demo::msg::Student;


// // 3.定义节点类；
// class MinimalPublisher : public rclcpp::Node
// {
//   public:
//     MinimalPublisher()
//     : Node("student_publisher"), count_(0)
//     {
//       // 3-1.创建发布方；在下面创建返回值，并接收这个publischer对象指针
//       publisher_ = this->create_publisher<Student>("topic_stu", 10);
//       // 3-2.创建定时器；
//       timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
//     }

//   private:
//     void timer_callback()
//     {
//       // 3-3.组织消息并发布。
//       auto stu = Student();
//       stu.name = "张三";
//       stu.age = count_++;
//       stu.height = 1.65;
//       RCLCPP_INFO(this->get_logger(), "学生信息:name=%s,age=%d,height=%.2f", stu.name.c_str(),stu.age,stu.height);
//       publisher_->publish(stu);

//     }
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<Student>::SharedPtr publisher_;
//     size_t count_;
// };

// int main(int argc, char * argv[])
// {
//   // 2.初始化 ROS2 客户端；
//   rclcpp::init(argc, argv);
//   // 4.调用spin函数，并传入节点对象指针。
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   // 5.释放资源；
//   rclcpp::shutdown();
//   return 0;
// }