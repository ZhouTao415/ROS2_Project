/* “Alt+Shift+A”
流程；
     1.包含头文件；
     2.初始化ROS2客户端；
     3.创建结点指针
     4.输出日志
     5.释放资源

 */


#include "rclcpp/rclcpp.hpp"


class AddIntsServer: public rclcpp::Node {
public: 
    AddIntsServer():Node("add_ints_Server_node_cpp"){ //构造函数， 设置名称
        RCLCPP_INFO(this->get_logger(), "服务端节点创建");
    }
};


int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 4. 调用spin函数，并传入节点对象指针
    rclcpp::spin(std::make_shared<AddIntsServer>());
    
    //  资源释放
    rclcpp::shutdown();
    return 0;
}