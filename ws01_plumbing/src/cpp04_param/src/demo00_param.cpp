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
class MyParam: public rclcpp::Node {
public: 
    MyParam():Node("my_param_node_cpp"){ //构造函数， 设置名称
        RCLCPP_INFO(this->get_logger(), "参数API使用");
        
        // 3-1.参数对象创建；
        rclcpp::Parameter p1("car_name", "volkswagen");
        rclcpp::Parameter p2("height", 1.68);
        rclcpp::Parameter p3("wheels", 4);
        // 3-2.参数对象的解析 (获取key，value, 将值转换成字符串...). 
        // 解析值
        // 当前值是字符串类型，as_string：我要获取你字符串的值，但是打印的化得转化为c风格的(c_str)
        RCLCPP_INFO(this->get_logger(), "car_name = %s", p1.as_string().c_str());
        RCLCPP_INFO(this->get_logger(), "height = %.2f", p2.as_double());
        RCLCPP_INFO(this->get_logger(), "wheels = %ld", p3.as_int()); // 返回int64的长整型用ld

        // 获取参数key 
        RCLCPP_INFO(this->get_logger(), "name = %s", p1.get_name().c_str());
        RCLCPP_INFO(this->get_logger(), "type = %s", p1.get_type_name().c_str());
        RCLCPP_INFO(this->get_logger(), "value2string = %s", p2.value_to_string().c_str());

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
    rclcpp::spin(std::make_shared<MyParam>());
    
    //  资源释放
    rclcpp::shutdown();
    return 0;
}