/* “Alt+Shift+A”
需求：创建客户端，组织数据并提交，然后处理响应结果（需要关注业务流程）。
流程；
    前提： main函数中需要判断提交的参数是否正确
    1.包含头文件；
    2.初始化ROS2客户端；
    3.自定义节点类
        3-1：创建客户端
        3-2：连接服务器（服务通信而言，如果客户端连接不到服务器，不能发送请求）
        3-3：发送请求
    4.创建对象指针
        需要调用连接服务的函数，根据连接结果，再做下一步处理；
        连接服务后，调用请求发送函数；
        再处理响应结果。
    5.释放资源

 */


#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;

class AddIntsClient: public rclcpp::Node {
public: 
    AddIntsClient():Node("add_ints_client_node_cpp"){ //构造函数， 设置名称
        RCLCPP_INFO(this->get_logger(), "客户端节点创建");
        // 3-1：创建客户端
        /* 
            模板： 服务接口
            参数：服务话题名称
            返回值：服务对象指针。
         */
        client_ = this->create_client<AddInts>("add_ints");
    }

    // 3-2. 等待服务连接
    // 区别于发布订阅：
    // 没有订阅方，发布的数据就丢掉了
    // 服务通信：数据比较重要，是在特定业务场景下，比如图像识别
    /* 
        连接服务器实现，如果成功返回 true,否则返回ufalse
     */
    bool connect_server(){
        // 我通过这个函数区连接服务器，我的超时时间为1s
        // 只要超时，我就返回false
        // 在指定超时时间内连接服务器，连接返回true，否false
        // client_->wait_for_service(1s);
        // 或者循环以1s为超时时间连接服务器，指导连接到服务器才推出循环。
        while (!client_->wait_for_service(1s))
        {   
            // 当前实现有问题，当按下Ctrl + C, 我的循环进入了死循环
            // 解决：
            // 对 Ctrl + C, 作出特殊处理
            // 1. 如何判断按下了Ctrl + C
            // 按下Ctrl + C 是结束 ROS2程序，意味着释放资源，比如：关闭context。
            // 2. 如何处理
            if (!rclcpp::ok()) //这个函数是判断是否在正常执行，正常返回true
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强行终止客户端！");
                return false;
            }
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接中");
        }
        
        return true;
    }

    // 3-3 发送请求：组织请求数据并发送响应
    // 编写发送请求函数。
    // 参数：两个整型数据
    // 返回值：提交请求后服务端的返回结果
    rclcpp::Client<AddInts>::FutureAndRequestId send_request(int num1, int num2){
        // 组织请求数据

        // 发送响应
        // rclcpp::Client<base_interfaces_demo::srv::AddInts>::FutureAndRequestId 
        // async_send_request(std::shared_ptr<base_interfaces_demo::srv::AddInts_Request> request) 
        // 相当于AddInts::Request对象指针
        auto request = std::make_shared<AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;

        return client_->async_send_request(request);
    }


private:
   
    rclcpp::Client<AddInts>::SharedPtr client_;
};


int main(int argc, char const *argv[])
{   
    // 判断argc 是否等于3
    // 客户端client在执行时候有三个参数
    // 参数1. 当前文件名字：demo02_client
    // 参数2 和 3 就是我们所提交的整型数据，
    // 目的：可以保证我们执行程序，可以动态提交两个整数
    if (argc != 3) {
        // 在类外调用log用下面方式，要其名字rclcpp
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交两个整型数数字！");
        return 1; 
    }
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 4. 调用spin函数，并传入节点对象指针
    // 1. 业务上没有必要回调挂起 
    // 2. 没有回调函数
    // rclcpp::spin(std::make_shared<AddIntsServer>());
    

    // 创建客户端的对象
    auto client = std::make_shared<AddIntsClient>();

    // 调用客户端对象的连接服务器功能
    bool flag = client->connect_server();

    if (!flag) {
        // rclcpp::get_logger("rclcpp") 创建logger 不依赖于context（一种容器）
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接失败，程序退出");
        // 'rcl node's context is invalid, at ./src/rcl/node.c:428'
        // 按下Ctrl + C 是结束 ROS2程序，意味着释放资源，比如：关闭context。
        // 已经释放了，所以会报错
        // RCLCPP_INFO(client->get_logger(), "服务器连接失败，程序退出");
    }
    
    // 调用请求提交函数，接受并处理响应结果。
    auto future = client->send_request(atoi(argv[1]), atoi(argv[2]));

    // 处理响应
    // 获取状态码，获取响应指针
    if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)//成功
    {
         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应成功！ sum = %d", future.get()->sum);
    }
    else // 失败
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应失败");
    }
    

    //  资源释放
    rclcpp::shutdown();
    return 0;
}

// #include "rclcpp/rclcpp.hpp"
// #include "base_interfaces_demo/srv/add_ints.hpp"

// using base_interfaces_demo::srv::AddInts;
// using namespace std::chrono_literals;

// class AddIntsClient: public rclcpp::Node {
// public: 
//     AddIntsClient():Node("add_ints_client_node_cpp"){ 
//         RCLCPP_INFO(this->get_logger(), "客户端节点创建");
//         client_ = this->create_client<AddInts>("add_ints");
//     }

//     bool connect_server(){
//         while (!client_->wait_for_service(1s))
//         {   
//             if (!rclcpp::ok()) 
//             {
//                 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强行终止客户端！");
//                 return false;
//             }  
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接中");
//         }
//         return true;
//     }

//     rclcpp::Client<AddInts>::FutureAndRequestId send_request(int num1, int num2){
//         auto request = std::make_shared<AddInts::Request>();
//         request->num1 = num1;
//         request->num2 = num2;
//         return client_->async_send_request(request);
//     }

// private:
//     rclcpp::Client<AddInts>::SharedPtr client_;
// };


// int main(int argc, char const *argv[])
// {   

//     if (argc != 3) {

//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交两个整型数数字！");
//         return 1; 
//     }

//     rclcpp::init(argc, argv);


//     auto client = std::make_shared<AddIntsClient>();

//     bool flag = client->connect_server();

//     if (!flag) {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接失败，程序退出");
//     }

//     auto future = client->send_request(atoi(argv[1]), atoi(argv[2]));


//     if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)
//     {
//          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应成功！ sum = %d", future.get()->sum);
//     }
//     else 
//     {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应失败");
//     }
    
//     rclcpp::shutdown();
//     return 0;
// }
