/* 
流程；
  需求：编写服务端，接收客户端发送请求，提取其中两个整型数据，相加后将结果响应回客户端。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建服务端；
      3-2.处理请求数据并响应结果。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。

 */

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using std::placeholders::_1;
using std::placeholders::_2;



// 3.定义节点类；
class AddIntsServer: public rclcpp::Node {
public: 
    AddIntsServer():Node("add_ints_server_node_cpp"){ //构造函数， 设置名称
        RCLCPP_INFO(this->get_logger(), "服务端节点创建");
        // 3-1.创建服务端；调用时候得传入两个参数，分别是请求和响应，那么我们设置下占位符
        /* 
          模板：服务接口  
          参数：
              1. 服务话题 （服务端，客户端。相同话题，绑定一起）
              2. 回调函数
          返回值：服务对象指针。
         */
        server_ = this->create_service<AddInts>("add_ints", std::bind(&AddIntsServer::add, this, _1, _2));
        
    }



    // 3-2.处理请求数据并响应结果

private:
    rclcpp::Service<AddInts>::SharedPtr server_;
    // 两个参数接口的请求和响应
    void add(const AddInts::Request::SharedPtr req, const AddInts::Response::SharedPtr res){
      res->sum = req->num1 + req->num2;
      RCLCPP_INFO(this->get_logger(), "%d + %d = %d", req->num1, req->num2, res->sum);
    }
};


int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 4. 调用spin函数，并传入节点对象指针
    rclcpp::spin(std::make_shared<AddIntsServer>());
    
    //  资源释放
    rclcpp::shutdown();
    return 0;
}