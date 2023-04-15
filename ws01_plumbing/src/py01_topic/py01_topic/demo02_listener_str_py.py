"""
    需求：订阅发布方发布的消息，并输出到终端。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建订阅方；
            3-2.处理订阅到的消息。
        4.调用spin函数,并传入节点对象;
        5.释放资源。
"""
# 1.导包；
import  rclpy
from rclpy.node import Node
from std_msgs.msg import String
#  3.定义节点类；
class Listener(Node):
    def __init__(self):
        # 继承的父类无参构造
        super().__init__("listener_node_py")
        self.get_logger().info("订阅方创建了(python)!")
        # 3-1.创建订阅方； self.do_cb: 处理回调函数
        #   参数
        #       1. 消息类型
        #       2. 话题名称
        #       3. 回调函数
        #       4. QOS(队列长度)
        #   返回值： 订阅对象
        self.subscription = self.create_subscription(String, "chatter", self.listener_callback, 10)

    # 回调函数执行的时候会被传入执行到的消息， msg 订阅到的消息
    def listener_callback(self, msg):
        # 3-2.处理订阅到的消息。
        self.get_logger().info("订阅的数据：%s" % msg.data)

def main():
    # 2.初始化 ROS2 客户端；
    rclpy.init()
    # 4.调用spin函数，并传入节点对象；
    rclpy.spin(Listener())
    # 5.释放资源。
    rclpy.shutdown()
if __name__ == '__main__':
    main()

