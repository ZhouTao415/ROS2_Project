/* 
流程；
  需求：演示参数API使用
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.参数对象创建；
      3-2.参数对象的解析 (获取key，value, 将值转换成字符串...).
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
 */

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;



// 3.定义节点类；
class MyParamClient: public rclcpp::Node {
public: 
    MyParamClient():Node("my_param_client_node_cpp"){ //构造函数， 设置名称
        RCLCPP_INFO(this->get_logger(), "参数客户端创建了");
        
        
    }

    // 3-2.处理请求数据并响应结果

private:
    // 两个参数接口的请求和响应
    
};


int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 4. 调用spin函数，并传入节点对象指针
    rclcpp::spin(std::make_shared<MyParamClient>());
    
    //  资源释放
    rclcpp::shutdown();
    return 0;
}