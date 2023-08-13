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
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Progress::Goal> goal){
      // uuid 与 终端出现的goal accepted with ID 是呼应的，为客户端生成一个独一无二的标识
      // 现在没用
      (void)uuid;
      

      // goal 可以拿到目标值，是个指针
      // 关于目标处理的，一般是接收或者拒绝
      // 业务逻辑： 判断提交的数字是否大于1,是就接收，否则就拒绝
      if(goal->num <= 1) {
        RCLCPP_INFO(this->get_logger(), "提交的目标之必须大于1!");
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(this->get_logger(),"提交的目标值合法!");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    // 3-3. 处理取消任务请求；（通过回调函数实现）
    /// Signature of a callback that accepts or rejects requests to cancel a goal.
    /*     
    using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    */
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handler){
      (void) goal_handler;
      RCLCPP_INFO(this->get_logger(), "接收的任务被取消！");
      return rclcpp_action::CancelResponse::ACCEPT;
    }


    // 3-4. 生成连续反馈和最终响应。（通过回调函数实现）
    /// Signature of a callback that is used to notify when the goal has been accepted.
    /*     
    using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    */
    // 这个过程是个耗时操作，为了避免主逻辑出现堵塞，单独开一个线程，来处理连续反馈和响应
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handler){

      // 1. 需要生成连续反反馈， 返回给客户端
      // void publish_feedback(std::shared_ptr<base_interfaces_demo::action::Progress_Feedback> feedback_msg)
      // goal_handler->publish_feedback();
      // 首先获取目标值，然后遍历，遍历中进行累加，且每循环一次就计算进度，并作为连续反馈发布

      int num = goal_handler->get_goal()->num;
      int sum = 0;
      auto feedback = std::make_shared<Progress::Feedback>();
      // 设置休眠 1.0 == 1HZ
      rclcpp::Rate rate(1.0);
      auto result = std::make_shared<Progress::Result>();

      for (int i = 1; i <= num; i++)
      {
        sum += i;

        // 累积过程中计算一个进度
        // 计算生成一个进度，设计进入Feedback
        double progess = i / (double)num;

        // 设计进入Feedback
        feedback->progress = progess;

        // 发布操作
        goal_handler->publish_feedback(feedback);
        
        RCLCPP_INFO(this->get_logger(), "连续反馈中，进度： %.2f", progess);

        // 解决取消任务时候的bug
        // 判断是否接收到了取消请求
        // goal_handler->is_canceling()
        // void canceled(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
        // goal_handler->canceled();
        if (goal_handler->is_canceling()) {
          
          // 如果接收到了就终止程序，return
          result->sum = sum;
          goal_handler->canceled(result);
          RCLCPP_INFO(this->get_logger(), "任务被取消!");
          return;
        }

        

        // 休眠一秒钟，为了显示是个耗时操作
        rate.sleep();
      }
      

      // 2. 生成最终响应结果
      // void succeed(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
      // goal_handler->succeed();

      if(rclcpp::ok()) {
        // 检验服务端是否正常运行
        result->sum = sum;
        // 把我们最终结果响应回去
        goal_handler->succeed(result);

        RCLCPP_INFO(this->get_logger(), "最终结果： %d", sum);
      }

    }

    void handle_accept(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handler){
      // 新建线程处理耗时主逻辑实现
      // 匿名线程,执行体,this 调用，传入参数 goal_handler
      std::thread(std::bind(&ProgressActionServer::execute, this, goal_handler)).detach();
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
