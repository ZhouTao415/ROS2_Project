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
using std::placeholders::_1;
using std::placeholders::_2;



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

      /*       
      返回值类型：std::shared_future<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>::SharedPtr> 
      函数名称：async_send_goal(
      const base_interfaces_demo::action::Progress::Goal &goal, 
      const rclcpp_action::Client<base_interfaces_demo::action::Progress>::SendGoalOptions &options)
      */
      auto goal = Progress::Goal();
      goal.num = num;
      rclcpp_action::Client<Progress>::SendGoalOptions options;
      options.goal_response_callback = std::bind(&ProgressActionClient::goal_response_callback, this, _1);
      options.feedback_callback = std::bind(&ProgressActionClient::feedback_callback, this,_1, _2);
      options.result_callback = std::bind(&ProgressActionClient::result_callback, this, _1);

      auto future = client_->async_send_goal(goal, options);
    }

    // 3-3. 处理关于目标值的服务端响应；回调函数
    /* 
    using GoalHandle = ClientGoalHandle<ActionT>;
    using GoalResponseCallback = std::function<void (typename GoalHandle::SharedPtr)>;
    */
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle){
        // 发动一个目标到服务端之后，服务端先拿到目标值，判断目标值是否可以被接收/拒绝
        // 然后将数据结果响应给我们的客户端
        // 如果目标值可处理 goal_handle 有内容
        // 如果不能被处理， 响应回的 goal_handle是个空指针
        if (!goal_handle) {
          RCLCPP_INFO(this->get_logger(), "目标请求被服务端拒绝");
        } else {
          RCLCPP_INFO(this->get_logger(), "目标处理中！");
        }
    }
    // 3-4. 处理连续反馈。回调函数
    /*  
        std::function<void (
        typename ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const Feedback>)>;
    */
    void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle, const std::shared_ptr<const Progress::Feedback> feedback){
        // 当服务端向客户端反馈信息时候，会调用 feedback_callback 函数
        // feedback 封装了反馈的数据
        // 这里只解析反馈的数据 goal_handle 用不上
        (void)goal_handle;
        // progress 就是我们获取的进度值
        double progress = feedback->progress;
        // 转化为百分比
        int pro = (int)(progress * 100);
        RCLCPP_INFO(this->get_logger(), "当前进度：%d%%", pro);
    }
    // 3-5. 处理最终响应。回调函数

    /*  
    std::function<void (const WrappedResult & result)>
    */

    void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult & result){
      // 当 feedback_callback 结束后,最终发送一个响应结果
      // 响应结果通过 result_callback 处理
      // 参数为 result ，即封装了
      // result.code： 通过状态码，可以反映你最终响应结果的一个状态
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        // result.result 具体类型是一个对响应对象的一个封装，封装成一个响应对象指针
        RCLCPP_INFO(this->get_logger(), "最终结果： %d", result.result->sum);
        
      } else if (result.code == rclcpp_action::ResultCode::ABORTED) 
      {
        RCLCPP_INFO(this->get_logger(), "被中断！");
      
      } else if (result.code == rclcpp_action::ResultCode::CANCELED)
      {
        RCLCPP_INFO(this->get_logger(), "被取消！");
        
      } else {
        RCLCPP_INFO(this->get_logger(), "未知异常！");

      }
      
      
      
      

    }
private:
    rclcpp_action::Client<Progress>::SharedPtr client_;
};

int main(int argc, char ** argv) {

  if (argc != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交一个整型数据！");
    return 1;
  }

  // void rclcpp::init(
  // int argc, 
  // const char *const *argv, 
  // const rclcpp::InitOptions &init_options = rclcpp::InitOptions(), 
  // rclcpp::SignalHandlerOptions signal_handler_options = rclcpp::SignalHandlerOptions::All)
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProgressActionClient>();
  node->send_goal(atoi(argv[1]));
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
