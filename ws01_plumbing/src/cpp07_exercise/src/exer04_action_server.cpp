/*
   需求：处理client请求发送的目标点，控制乌龟向该目标点运动，并连续反馈乌龟与目标点之间的剩余距离。
   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.创建原生乌龟位姿订阅方，回调函数中获取乌龟位姿；
            3-2.创建原生乌龟速度发布方；
            3-3.创建动作服务端；
            3-4.解析动作客户端发送的请求；
            3-5.处理动作客户端发送的取消请求；
            3-6.实现主逻辑(耗时操作),创建新线程处理请求; (启动子线程)
            3-7.子线程中，发送速度指令，新线程产生连续反馈并响应最终结果。
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。
*/


#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using base_interfaces_demo::action::Nav;


class Exer04_action_server : public rclcpp::Node{ 
public:
    Exer04_action_server() : Node("exer04_action_server_node_cpp"), turtle1_x(0.0), turtle1_y(0.0) {  
      // RCLCPP_INFO(this->get_logger(), "动作服务端!");
      // 3-1.创建原生乌龟位姿订阅方，回调函数中获取乌龟位姿；
      sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&Exer04_action_server::poseCallBack, this, _1));
      // 3-2.创建原生乌龟速度发布方； this 是个节点，其创建通信对象
      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10); 
      // 3-3.创建动作服务端；
      /* 
        NodeT node, 
        const std::string &name, 
        rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
        rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
        rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted 
        
      */
      action_sever_ = rclcpp_action::create_server<Nav>(
        this,
        "nav",
        std::bind(&Exer04_action_server::handle_goal, this, _1, _2),
        std::bind(&Exer04_action_server::handle_cancel, this, _1),
        std::bind(&Exer04_action_server::handle_accepted, this, _1)
      );
      // 3-4.解析动作客户端发送的请求；
      // 3-5.处理动作客户端发送的取消请求；
      // 3-6.实现主逻辑(耗时操作),创建新线程处理请求; (启动子线程)
      // 3-7.子线程中，发送速度指令，新线程产生连续反馈并响应最终结果。
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_action::Server<Nav>::SharedPtr action_sever_;

    float turtle1_x, turtle1_y;
    void poseCallBack(const turtlesim::msg::Pose::SharedPtr pose) {
        // x,y 的目的是为了接收变量
        turtle1_x = pose->x;
        turtle1_y = pose->y;
    }
    // 请求目标处理
    // GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Nav::Goal> goal) {
      (void) uuid;
      // (void) goal;
      //  取出目标中的x, y坐标，分别判断是否超出 [0, 11.08] 范围， 如果超出，认为非法，否则合法；
      if (goal->goal_x < 0 || goal->goal_x > 11.08 || goal->goal_y < 0 || goal->goal_y > 11.08) {
        RCLCPP_INFO(this->get_logger(), "目标点超出正常取取值范围!");
        return rclcpp_action::GoalResponse::REJECT;
      }
      RCLCPP_INFO(this->get_logger(), "目标点合法!");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // 取消请求处理
    // CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>) ActionT: action type
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle) {
      (void) goal_handle;
      RCLCPP_INFO(this->get_logger(), "取消任务!");
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    // 主逻辑处理
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle) {
      // 子线程处理主要逻辑
      RCLCPP_INFO(this->get_logger(), "主逻辑开始执行!");
      
      // 创建对象
      // 用来封装最终结果的,用于下面的1
      
      auto result = std::make_shared<Nav::Result>(); //最终结果对象
      auto feedback = std::make_shared<Nav::Feedback>(); //连续反馈对象
      geometry_msgs::msg::Twist twist; //速度指令对象

      // 1. 生成连续反馈
      rclcpp::Rate rate(1.0); //控制反馈的频率

      while (true)
      {
        // 1. 如果client 要取消任务，要特殊处理
        if (goal_handle->is_canceling())
        {
          // 即使取消，也要有最终结果对象 
          // 设置取消后的最终结果
          goal_handle->canceled(result);
          return;
        }
        

        // 2. 解析目标点坐标，与原生乌龟的实时坐标
        float goal_x = goal_handle->get_goal()->goal_x;
        float goal_y = goal_handle->get_goal()->goal_y;


        // 3. 解析后，计算剩余距离，并发布
        float distance_x = goal_x - turtle1_x;
        float distance_y = goal_y - turtle1_y;
        float distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
        // 发布 publish
        // 把feedback 中feedback 赋值
        feedback->distance = distance;
        goal_handle->publish_feedback(feedback);

      // 2. 发布乌龟运动指令 (在1中实现， 用到循环，休眠)
        // 4. 根据剩余距离，计算速度指令，并发布
        // 离得远些速度快，离的近些，速度慢
        float scale = 0.5;
        float linear_v_x = scale * distance_x;
        float linear_v_y = scale * distance_y;
        // 发布数值
        twist.linear.x = linear_v_x;
        twist.linear.y = linear_v_y;
        cmd_pub_->publish(twist);
        
        // 5. 循环结束条件，比如剩余距离为0， 但是实际情况不行
        //    设计一个阈值之内
        if (distance <= 0.05)
        {
          // 与目标点的剩余距离小于0.05m，结束导航
          RCLCPP_INFO(this->get_logger(), "乌龟已导航至目标点!");
          break;
        }
        
        // 休眠
        rate.sleep();
      }

      // 3. 生成最响应结果
      if (rclcpp::ok())
      {
        result->turtle_x = turtle1_x;
        result->turtle_y = turtle1_y;
        goal_handle->succeed(result);
      }
      
    }

    // void (std::shared_ptr<ServerGoalHandle<ActionT>)
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle) {
      (void) goal_handle;
      // 主逻辑是个耗时操作,需要个线程,以防止主线程堵塞,要开启子线程
      // C++ 开启子线程
      std::thread(std::bind(&Exer04_action_server::execute, this, goal_handle)).detach();
    }
    
};

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Exer04_action_server>());
  rclcpp::shutdown();

  return 0;
}
