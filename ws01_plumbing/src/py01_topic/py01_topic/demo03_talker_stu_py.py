"""
# 需求:
# 步骤：
#   1.导包；
#   2.初始化 ROS2 客户端；
#   3.定义节点类；
#   4.调用spin函数,并传入自定义类对象；
#   5.释放资源.


"""
#   1.导包；
import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

#   3.定义节点类；
# 继承 Node
class TalkerStu(Node):
    def __init__(self):
        # 创建一个父节点，父类没有提供无参构造，设置节点名称
        super().__init__("talkerStu_node_py")
        stu = Student

def main():
    # 2.初始化 ROS2 客户端；
    rclpy.init()
    # 4.调用spin函数，并传入自定义类对象；
    rclpy.spin(TalkerStu())
    # 5.释放资源.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
