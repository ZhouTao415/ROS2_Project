/* 
流程；
  需求：创建参数客户端，查询或修改i服务端参数
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1. 创建参数客户端对象；
      3-2. 连接服务端
      3-3. 参数查询
      3-4. 参数的修改
    4.创建自定义节点对象，并调用其函数实现
    5.释放资源。
 */

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;



// 3.定义节点类；
class MyParamClient: public rclcpp::Node {
public: 
    MyParamClient():Node("my_param_client_node_cpp"){ //构造函数， 设置名称
        RCLCPP_INFO(this->get_logger(), "参数客户端创建了");
        // 3-1. 创建参数客户端对象；
        // 参数1：当前对象所依赖的节点
        // 参数2：参数服务端节点名称
        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "my_param_server_node_cpp");
        /* 
        
        Question: 服务通信不是通过服务话题关联么？ 
                  为什么参数客户端是通过参数服务端的节点来关联?
        Answer:
                1. 参数服务端启动后，底层封装了多个服务通信的服务端
                2. 每个服务端的话题，都采用了 /服务端节点名称/ xxx;
                3. 参数客户端创建后， 也会封装多个服务通信的客户端；
                4. 这些客户端与服务端相呼应，也要使用相同的话题，
                   因此客户端在创建时需要使用服务端节点名称
                   
                   ros2 service list 
                   
                   /my_param_server_node_cpp/describe_parameters
                   
                   /my_param_server_node_cpp/get_parameter_types
                   
                   /my_param_server_node_cpp/get_parameters
                   
                   /my_param_server_node_cpp/list_parameters
                   
                   /my_param_server_node_cpp/set_parameters
                   
                   /my_param_server_node_cpp/set_parameters_atomically
        */
        
    }

      // 3-2. 连接服务端
      bool connect_server(){
        while (!param_client_->wait_for_service(1s))
        {
          if (!rclcpp::ok()){
            return false;
          }
          
          RCLCPP_INFO(this->get_logger(), "服务连接中...");
        }

        return true;
      }
      // 3-3. 参数查询
      void get_param(){
          RCLCPP_INFO(this->get_logger(), "------------参数查询-----------");
          // 获取某个参数
          std::string car_name = param_client_->get_parameter<std::string>("car_name");
          double width = param_client_->get_parameter<double>("width");
          RCLCPP_INFO(this->get_logger(), "car_name = %s", car_name.c_str());
          RCLCPP_INFO(this->get_logger(), "width = %.2f", width);

          // 获取多个参数
          auto params = param_client_->get_parameters({"car_name", "width", "wheels"});

          for (auto &&param : params){
            RCLCPP_INFO(this->get_logger(), "%s = %s", param.get_name().c_str(), param.value_to_string().c_str());
          }
          // 判断是否包含参数

          RCLCPP_INFO(this->get_logger(), "包含car_name么? %d", param_client_->has_parameter("car_name"));
          RCLCPP_INFO(this->get_logger(), "包含height么? %d", param_client_->has_parameter("height"));
          
      }
      // 3-4. 参数修改
      void update_param(){
          RCLCPP_INFO(this->get_logger(), "------------参数修改-----------");
          param_client_->set_parameters({rclcpp::Parameter("car_name", "BMW"),
            rclcpp::Parameter("width, 3.0"),
            rclcpp::Parameter("length", 5.0) //不存在的参数 ,前面要设置为true  rclcpp::NodeOptions().allow_undeclared_parameters(true)      
          });
          RCLCPP_INFO(this->get_logger(), "新设置的参数:%.2f", param_client_->get_parameter<double>("length"));   
      }

private:
    rclcpp::SyncParametersClient::SharedPtr param_client_;
    
};


int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);

    auto clinet = std::make_shared<MyParamClient>();
    bool flag = clinet->connect_server();
    if(!flag){
      return 0;
    }

    clinet->get_param();
    clinet->update_param();
    // 修改完后，再次查询
    clinet->get_param();

    // 4. 调用spin函数，并传入节点对象指针
    // rclcpp::spin(std::make_shared<MyParamClient>());
    
    //  资源释放
    rclcpp::shutdown();
    return 0;
}