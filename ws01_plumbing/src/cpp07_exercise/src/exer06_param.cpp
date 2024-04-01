/*
   需求：修改turtlesim_node的背景颜色。
   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.创建参数客户端；
            3-2.连接参数服务端；
            3-3.更新参数。
       4.创建对象指针,并调用其函数；
       5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Exer06Param : public rclcpp::Node{ 
public:

    Exer06Param() : Node("exer06_param_node_cpp") {  
    //   3-1.创建参数客户端；
    client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/turtlesim");
      
    }
    //   3-2.连接参数服务端；
    bool connect_server() {
        //  没有连接成功
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "客户端强制退出!");
                return false;
            }
            
            RCLCPP_INFO(this->get_logger(), "服务连接中!");
            
        }
        return true;
    }
    //   3-3.更新参数。
    void update_param() {
        //  背景色递进的方式修改
        // background_r [0, 255] 5为步进， 0->255->0
        // 1. 获取参数, 
        // 通过key 获得value
        red = client_->get_parameter<int32_t>("background_r");
        // 2. 编写循环，修改参数（通过休眠，来修改频率）
        rclcpp::Rate rate(30.0);
        /* 
            需求:背景色渐变，由浅变深，反之亦然
            [0, 255]->[255, 0]
            实现：
                1. 一个完成周期，技术是511,中间数值是255
                2. 创建一个计数器，初始数值和red相同，递增的步进数值也和red相同，取值是[0,511]
                3. 当计数器在[0, 255]之间，递增， 在[256, 511]递减；
                4. 当计数器>= 511时候，归零
        */
        int count = red;
        while (rclcpp::ok())
        {
            // red += 5;   
            (count <= 255) ? red += 5 : red -= 5;
            count += 5;

            if (count > 511)
            {
                count = 0;
            }
            
            // 修改服务端参数
            client_->set_parameters({rclcpp::Parameter("background_r", red)});
            rate.sleep();

        }
        
    }

private:
    rclcpp::SyncParametersClient::SharedPtr client_;
    int32_t red;

};

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<Exer06Param>());
    auto client_ = std::make_shared<Exer06Param>();
    if (!client_->connect_server())
    {
        return 1;
    }
    // 调用其函数
    client_->update_param();
    rclcpp::shutdown();

    return 0;
}