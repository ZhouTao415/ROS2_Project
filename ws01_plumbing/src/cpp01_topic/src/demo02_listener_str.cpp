/*  
    需求：订阅发布方发布的消息，并输出到终端。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建订阅方；
            3-2.处理订阅到的消息。
        4.调用spin函数，并传入节点对象指针；
        5.释放资源。
*/

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 3.自定义定义节点类；
class Listener: public rclcpp::Node {
public:
    // Listener是一个构造函数
    // Constructors, initialize the Node: listerner_node_cpp
    Listener():Node("listerner_node_cpp") {
        RCLCPP_INFO(this->get_logger(), "creat the subscriber");
        // 3-1.创建订阅方；

        /* 
            模板： 消息类型；
            参数： 
                  1. 话题名称；
                  2. QOS，队列长度；
                  3. 绑定一个回调函数，在回调函数中处理订阅到的数据
            返回值： 订阅对象指针；
         */
        subscription_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&Listener::do_cb, this, std::placeholders::_1));
        
    }
private:
    void do_cb(const std_msgs::msg::String &msg){
        // 3-2.处理订阅到的消息。
        RCLCPP_INFO(this->get_logger(), "Subscribed messages: %s", msg.data.c_str());
    }
    // 定义一个成员变量来接收
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;


};  


int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);

    // 4.调用spin函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<Listener>());


    // 5.释放资源。
    rclcpp::shutdown();
    
    return 0;
}
