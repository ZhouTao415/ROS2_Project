from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 1. 启动两个 turtlesim_node, 其中一个要设置命名空间，处理topic, node 重名的情况
    t1 = Node(package = "turtlesim", executable = "turtlesim_node")
    t2 = Node(package = "turtlesim", executable = "turtlesim_node", namespace = "t2")
    
    # 2. 控制第二个乌龟掉头
    rotate = ExecuteProcess(
        # 存储一些要被执行的终端指令
        cmd = ["ros2 action send_goal /t2/turtle1/rotate_absolute turtlesim/action/RotateAbsolute \"{'theta': 3.14}\""],
        # 日志输出位置, both: 既输出到日至文件，也输出到terminal
        output = "both",
        # 设置成bool， 参数值必须设置成终端指令来执行
        shell =  True
    )
    # 3. 调用自定义的节点，并且该节点调用顺序有要求(要在掉头完毕后才执行)
    exer01 = Node(package = "cpp07_exercise", executable = "exer01_pub_sub")
    # 怎么控制节点的执行顺训，需要通过注册事件完成
    # 创建事件注册对象，对象当中声明哪个目标节点，在哪个事件触发时候，执行哪种操作？
    register_roate_exit_event = RegisterEventHandler(
        # 创建一个新对象
        event_handler = OnProcessExit( # 触发动作 
            # 目标节点
            target_action = rotate,
            # 触发执行的事件
            on_exit = exer01
        )
    )


    # LaunchDescription对象，对象有列表，列表由多个node组成或其他
    return LaunchDescription([t1, t2, rotate, register_roate_exit_event])