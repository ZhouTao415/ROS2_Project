/*
   需求：向动作服务端发送目标点数据，并处理服务端的响应数据。
   步骤：
       0.解析launch 文件传入的参数
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.创建动作客户端；
            3-2.连接服务端，发送请求数据，并处理服务端响应；
            3-3.处理目标响应；
            3-4.处理响应的连续反馈；
            3-5.处理最终响应。
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。
*/


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"

using namespace std::chrono_literals;
using base_interfaces_demo::action::Nav;
using std::placeholders::_1;
using std::placeholders::_2;

class Exer05_action_client : public rclcpp::Node{ 
public:

    Exer05_action_client():Node("exer05_action_client_node_cpp") {  
      RCLCPP_INFO(this->get_logger(), "动作客户端创建!");
      // 3-1.创建动作客户端；
      client_ = rclcpp_action::create_client<Nav>(this, "nav");
    }
      // 3-2.连接服务端，发送请求数据，并处理服务端响应；
      void send_goal(float goal_x, float goal_y, float goal_theta) {
      // 1. 连接服务端
      if (!client_->wait_for_action_server(10s))
      {
        RCLCPP_INFO(this->get_logger(), "服务连接超时!");
        return;
      }
      
      // 2. 组织并发送数据,绑定回调函数
      // const base_interfaces_demo::action::Nav::Goal &goal, 
      // const rclcpp_action::Client<base_interfaces_demo::action::Nav>::SendGoalOptions &options
      Nav::Goal goal;
      goal.goal_x = goal_x;
      goal.goal_y = goal_y;
      goal.goal_theta = goal_theta;

      rclcpp_action::Client<Nav>::SendGoalOptions options;
      // options需要绑定回调函数
      // std::function<void (std::shared_ptr<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Nav>>)> 
      options.goal_response_callback = std::bind(&Exer05_action_client::goal_response_callback, this, _1);
      // std::function<...> rclcpp_action::Client<...>::SendGoalOptions::feedback_callback
      options.feedback_callback = std::bind(&Exer05_action_client::feedback_callback, this, _1, _2);
      // std::function<void (const rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Nav>::WrappedResult &result)> 
      options.result_callback = std::bind(&Exer05_action_client::result_callback, this, _1);
      client_->async_send_goal(goal, options);
      }
      // 3-3.处理目标响应；
      void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<Nav>> goal_handler) {
        // 如果拒绝处理，那么goal_handler == nullptr
        if (!goal_handler)
        {
          RCLCPP_INFO(this->get_logger(), "请求目标非法!");
        } else
        {
          RCLCPP_INFO(this->get_logger(), "目标值被接收!");
        }
        
        
      }
      // 3-4.处理响应的连续反馈；
      void feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<Nav>> goal_handler, std::shared_ptr<const Nav::Feedback> feedback) {
        (void) goal_handler;
        RCLCPP_INFO(this->get_logger(), "剩余 %.2f m", feedback->distance);
      };
      // 3-5.处理最终响应。
      void result_callback(const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
          // 响应成功
          RCLCPP_INFO(this->get_logger(), "乌龟最终位姿信息， 坐标: (%.2f, %.2f), 航向: %.2f.", 
                      result.result->turtle_x, result.result->turtle_y, result.result->turtle_theta);
        } else {
          // 响应失败
          RCLCPP_INFO(this->get_logger(), "响应失败!");
        }
        
      }

private:
    rclcpp_action::Client<Nav>::SharedPtr client_;
};

int main(int argc, char ** argv) {
  // 0.解析launch 文件传入的参数
  if (argc != 5)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "请输入合法的目标点数据");
    return 1;
  }
  
  rclcpp::init(argc, argv);
  // 不能直接spin， 需要调用send_goal 函数
  auto client = std::make_shared<Exer05_action_client>();
  // 把string 类型专float
  client->send_goal(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  rclcpp::spin(client);
  rclcpp::shutdown();

  return 0;
}
