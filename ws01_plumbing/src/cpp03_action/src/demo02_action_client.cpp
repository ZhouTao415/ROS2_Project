/* 
 需求：编写动作客户端，可以反宋一个整型数据到服务端，并处理服务端连续反馈和最终响应结果
 步骤：
 前提： 可以解析终端下动态传入的参数
   1.包含头文件；
   2.初始化 ROS2 客户端；
   3.定义节点类；
     3-1.创建发动作客户端；
     3-2. 发送请求
     3-3. 处理关于目标值的服务端响应；回调函数
     3-4. 处理连续反馈。回调函数
     3-5. 处理最终响应。回调函数
   4.调用spin函数，并传入节点对象指针；
   5.释放资源.

 */


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using base_interfaces_demo::action::Progress;
using namespace std::chrono_literals;

class ProgressActionClient : public rclcpp::Node{ 
public:

    ProgressActionClient():Node("progress_action_client_node_cpp"){  
      RCLCPP_INFO(this->get_logger(), "action client creation!");
      // 3-1.创建发动作客户端；
      // rclcpp_action::Client<ActionT>::SharedPtr //返回值
      // create_client<ActionT, NodeT>(NodeT node, 
      // const std::string &name, 
      // rclcpp::CallbackGroup::SharedPtr group = nullptr, 
      // const rcl_action_client_options_t &options = rcl_action_client_get_default_options())
      client_ = rclcpp_action::create_client<Progress>(
        this, 
        "get_sum");
    }
    // 3-2. 发送请求
    void send_goal(int num){
      // 1.需要连接服务端
      // 超时时间设置为10s
      if (!client_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(this->get_logger(), "服务器连接失败！");
        return;
      }
      // 2.发送具体请求
      
    }

    // 3-3. 处理关于目标值的服务端响应；回调函数
    // 3-4. 处理连续反馈。回调函数
    // 3-5. 处理最终响应。回调函数
private:
    rclcpp_action::Client<Progress>::SharedPtr client_;
};

int main(int argc, char ** argv) {

  if (argc != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交一个整型数据！");
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProgressActionClient>();
  node->send_goal(atoi(argv[1]));
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
