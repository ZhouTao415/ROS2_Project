/* “Alt+Shift+A”
流程；
     1.包含头文件；
     2.初始化ROS2客户端；
     3.自定义节点类
     
     4.创建对象指针
     5.释放资源

 */


#include "rclcpp/rclcpp.hpp"


class AddIntsClient: public rclcpp::Node {
public: 
    AddIntsClient():Node("add_ints_client_node_cpp"){ //构造函数， 设置名称
        RCLCPP_INFO(this->get_logger(), "客户端节点创建");
    }
};


int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 4. 调用spin函数，并传入节点对象指针
    // 1. 业务上没有必要回调挂起 
    // 2. 没有回调函数
    // rclcpp::spin(std::make_shared<AddIntsServer>());
    

    // 创建客户端的对象
    auto client = std::make_shared<AddIntsClient>();

    //  资源释放
    rclcpp::shutdown();
    return 0;
}