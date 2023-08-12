/* 
 需求：编写动作服务端，需要解析客户端提交的数字，遍历该数字并累加求和，最终结果响应回客户端，
      且请请求响应过程中需要生成连续反馈。
 分析：
      1. 创建动作服务端；
      2. 处理请求数据/提交的目标值；
      3. 生成连续反馈。
      4. 响应最终结果
      5. 处理取消任务请求；
      
 步骤：
   1.包含头文件；
   2.初始化 ROS2 客户端；
   3.定义节点类；
        3-1. 创建动作服务端；
        3-2. 处理提交的目标值；（通过回调函数实现）
        3-3. 处理取消任务请求；（通过回调函数实现）
        3-4. 生成连续反馈和最终响应。（通过回调函数实现）

   4.调用spin函数，并传入节点对象指针；
   5.释放资源.

 */


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;

class ProgressActionServer : public rclcpp::Node{ 
public:

    ProgressActionServer():Node("progress_action_server_node_cpp"){  
      RCLCPP_INFO(this->get_logger(), "action server creation!");

      // 3-1. 创建动作服务端；

      /*   
      rclcpp_action::Server<ActionT>::SharedPtr //返回值
      create_server<ActionT, NodeT> //消息类型模板
      (NodeT node, //节点
      const std::string &name, //话题名称

      // 三个回调函数
      rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
      rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
      rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted, 
      
      // 下面有默认的参数值
      const rcl_action_server_options_t &options = rcl_action_server_get_default_options(), 
      rclcpp::CallbackGroup::SharedPtr group = nullptr)
       */
      action_server_ = rclcpp_action::create_server<Progress>(
          this, 
          "get_sum",
          // 易犯错误：调用函数是有参数的，需要占位符,几个参数需要几个占位符
          std::bind(&ProgressActionServer::handle_goal, this, _1,_2),
          std::bind(&ProgressActionServer::handle_cancel, this, _1),
          std::bind(&ProgressActionServer::handle_accept, this, _1)
          );

    }
    // 3-2. 处理提交的目标值；（通过回调函数实现）
    /// Signature of a callback that accepts or rejects goal requests.
    /*     
    using GoalCallback = std::function<GoalResponse(
        const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Progress::Goal>){
      // 关于目标处理的，一般是接收或者拒绝
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    // 3-3. 处理取消任务请求；（通过回调函数实现）
    /// Signature of a callback that accepts or rejects requests to cancel a goal.
    /*     
    using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    */
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> go_handler){
      
      return rclcpp_action::CancelResponse::ACCEPT;
    }


    // 3-4. 生成连续反馈和最终响应。（通过回调函数实现）
    /// Signature of a callback that is used to notify when the goal has been accepted.
    /*     
    using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    */
   void handle_accept(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>>go_handler){
    
   }




private:
  rclcpp_action::Server<Progress>::SharedPtr action_server_;

};

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProgressActionServer>());
  rclcpp::shutdown();

  return 0;
}
