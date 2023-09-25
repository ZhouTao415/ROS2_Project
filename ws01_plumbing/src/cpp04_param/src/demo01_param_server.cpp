/* 
流程；
  需求：创建参数服务端并操作数据（增删改查）
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1. 增
      3-2. 查
      3-3. 改
      3-4. 删
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
 */

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;



// 3.定义节点类；
class MyParamServer: public rclcpp::Node {
public: 
    // 为了安全操作，需要对删除进行声明操作
    // 如果允许参数删除，那么需要通过NodeOptions 声明
    // 一个普通的节点就可以作为参数服务端存在
    MyParamServer():Node("my_param_server_node_cpp", rclcpp::NodeOptions().allow_undeclared_parameters(true)){ //构造函数， 设置名称
        RCLCPP_INFO(this->get_logger(), "参数服务端创建了");

    }
    // 3-1. 增
    void declare_param() {
      RCLCPP_INFO(this->get_logger(), "------------------增---------------");
      this->declare_parameter("car_name", "volkswagen");
      this->declare_parameter("width", 1.55);
      this->declare_parameter("wheels", 4);
    }
    // 3-2. 查
    void get_param() {
      RCLCPP_INFO(this->get_logger(), "------------------查---------------");
      // this->get_parameter();
      // this->get_parameters();
      // this->has_parameter();

      // 获取指定参数
      auto car = this->get_parameter("car_name");
      RCLCPP_INFO(this->get_logger(), "key = %s, value = %s", car.get_name().c_str(), car.as_string().c_str());
      // 获取一些参数
      auto params = this->get_parameters({"car_name", "width", "wheels"});
      for (auto &&param : params)
      {
        RCLCPP_INFO(this->get_logger(), "(%s = %s)", param.get_name().c_str(), param.value_to_string().c_str());
      }

      // 判断是否包含
      RCLCPP_INFO(this->get_logger(), "是否包含car_name? %d", this->has_parameter("car_name"));
      RCLCPP_INFO(this->get_logger(), "是否包含width? %d", this->has_parameter("width"));
      RCLCPP_INFO(this->get_logger(), "是否包含height? %d", this->has_parameter("height"));


    }
    // 3-3. 改
    void update_param() {
      RCLCPP_INFO(this->get_logger(), "------------------改---------------");
    }
    // 3-4. 删
    void del_param() {
      RCLCPP_INFO(this->get_logger(), "------------------删---------------");
    }

private:
    // 两个参数接口的请求和响应
    
};


int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 4. 调用spin函数，并传入节点对象指针

    auto node = std::make_shared<MyParamServer>();
    node->declare_param();
    node->get_param();
    node->update_param();
    node->del_param();
    
    rclcpp::spin(node);
    
    //  资源释放
    rclcpp::shutdown();
    return 0;
}