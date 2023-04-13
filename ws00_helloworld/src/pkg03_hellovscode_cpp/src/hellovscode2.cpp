/* “Alt+Shift+A”
流程；
     1.包含头文件；
     2.初始化ROS2客户端；
     3.创建结点指针
     4.输出日志
     5.释放资源

 */
#include "rclcpp/rclcpp.hpp"
int main(int argc, char **argv){
    //  2.初始化ROS2客户端
    rclcpp::init(argc, argv);//
    //  3.创建结点指针
        //rclcpp 有个Node类，Node类有个函数，之后给u节点起个名字
    auto node = rclcpp::Node::make_shared("helloworld_node_cpp");
    //  4.输出日志
    RCLCPP_INFO(node->get_logger(), "hello world!(pycharm) .... ++++");
    //  5.释放资源
    rclcpp::shutdown();

    return 0;
}