#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node{ 
public:

    Talker():Node("time_node_cpp") {  
      // 演示Rate的使用
      
    }

private:

    void demo_rate(){

    }


};

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();

  return 0;
}