/* 
 需求：以某个固定频率发送文本“hello world!”，文本后缀编号，每发送一条消息，编号递增1。
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
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node{ 
public:

    Talker():Node("talker_node_cpp"), count_(0){  
      RCLCPP_INFO(this->get_logger(), "发布节点创建！");
      RCLCPP_INFO(this->get_logger(), "作者: TAO");
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);  
      timer_  = this->create_wall_timer(500ms, std::bind(&Talker::on_timer, this));
    }

private:

    void on_timer(){
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "发布的消息：'%s'", message.data.c_str()); 
      publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_; 
};

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();

  return 0;
}




// // 1.包含头文件；
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// using namespace std::chrono_literals; //设置持续时间，用一个自变量后缀一个时间单位 1s 100ms
// // 3.自定义节点类；
// class Talker : public rclcpp::Node{ // 继承rclcpp 下面的Node
// public:

//     // 设置构造函数：Talker ，父类继承：Node， 传入节点名称：talker_node_cpp count_ 初始化为 0
//     Talker():Node("talker_node_cpp"), count_(0){  

//       // 打印
//       RCLCPP_INFO(this->get_logger(), "发布节点创建！");

//       // 3-1.创建发布方；Qos 服务；模板：消息类型质量管理,队列最多放10条
//           /* 
//             模板： 被发布的消息类型；
//             参数:
//               1. 话题名称;
//               2. Qos(消息队列长度):最多方十条，当网络堵塞
//             返回值:发布对象指针
//           */
//       publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

 
//       // 3-2.创建定时器；控制发布频率 (时间，回调函数bind 绑定一下, 调用当前类下的 on_timer ,this 调用)
               
//               参数：
//                 1. 时间间隔
//                 2. 回调函数 call back
//               返回值：定时器对象指针
//               绑定一下，回调函数要调用当前类下， this 来调用
//               */      
//       timer_  = this->create_wall_timer(500ms, std::bind(&Talker::on_timer, this));
//     }



//     // 3-1 声明一个成员变量 上面去接收这个变量的返回值
// private:
//     //回调函数无参数无返回值
//       void on_timer(){
//         //      3-3.组织消息并发布。
//         //创建一个message 对象
//         auto message = std_msgs::msg::String();
//         // String 里面有字段 data
//         message.data = "Hello, world! " + std::to_string(count_++);
//         RCLCPP_INFO(this->get_logger(), "发布的消息：'%s'", message.data.c_str()); //转化为 c string
//         publisher_->publish(message);
//       }
//       这行代码定义了一个名为publisher_的ROS 2发布者指针，该发布者用于发布std_msgs::msg::String类型的消息
//       rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//       rclcpp::TimerBase::SharedPtr timer_;
//       size_t count_; //计数器

// };




// int main(int argc, char ** argv)
// {
// //  2.初始化 ROS2 客户端；
//     rclcpp::init(argc, argv);

// //    3.定义节点类；
// //      3-1.创建发布方；
// //      3-2.创建定时器；控制发布频率
// //      3-3.组织消息并发布。
// //    4.调用spin函数，并传入节点对象指针； 
// //      执行到这之后，回头了，回头去等着上面class Talker对象里面的回调函数，
// //      但是现在什么都没有，只是一个挂起状态
//     // make_shared 在动态内存中分配一个对象并初始化它，返回指向此对象的shared_ptr，与智能指针一样
//     rclcpp::spin(std::make_shared<Talker>());

// //  5.释放资源.
//     rclcpp::shutdown();

//   return 0;
// }

