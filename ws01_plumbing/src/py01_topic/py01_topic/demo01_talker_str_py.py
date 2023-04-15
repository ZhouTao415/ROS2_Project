# 需求：以某个固定频率发送文本“hello world!”，文本后缀编号，每发送一条消息，编号递增1。
# 步骤：
#   1.导包；
#   2.初始化 ROS2 客户端；
#   3.定义节点类；
#     3-1.创建发布方；
#     3-2.创建定时器；控制发布频率
#     3-3.组织消息并发布。
#   4.调用spin函数，并传入自定义类对象；
#   5.释放资源.


#   1.导包；
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 继承 Node
class Talker(Node):
    def __init__(self):
        # 创建一个父节点，父类没有提供无参构造，设置节点名称
        super().__init__("talker_node_py")
        self.get_logger().info("发布方创建(python)!")
        # 设置计数器
        self.count = 0
        # 3-1.创建发布方； 类型，话题名称，qos 等待几条
        #   参数
        #       1. 消息类型
        #       2. 话题名称
        #       3. QOS,队列长度
        #   返回值： 发布对象
        self.publisher = self.create_publisher(String, "chatter", 10)
        #  3-2.创建定时器；控制发布频率 self.on_timer:回调函数
        #   参数
        #       1. 时间间隔
        #       2. 回调函数
        #   返回值： 定时器对象
        selftimer = self.create_timer(1.0, self.on_timer)


    def on_timer(self):
        #  3-3.组织消息并发布
        message = String()
        message.data = "hello_world (python)" + str(self.count)
        self.publisher.publish(message)
        self.count += 1
        # 为了在发布方看到写出的数据是什么样子的
        self.get_logger().info("发布的数据%s" % message.data)

def main():
    # 2.初始化 ROS2 客户端；
    rclpy.init()
    # 4.调用spin函数，并传入自定义类对象；
    rclpy.spin(Talker())
    # 5.释放资源.
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    main
