from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
def generate_launch_description():
    # 所需变量需要封装
    x = 8.54
    y = 9.54
    theta = 0.0
    turtle_name = "t2"
    # 1. 在目标点处生成一个新乌龟
    # ros2 service call /spawn turtlesim/srv/Spawn "{'x': 6.0, 'y': 9.0, 'theta': 0.0, 'name': 't2'}"
    spawn = ExecuteProcess(
        cmd = ["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': " + str(x) + 
               ", 'y': " + str(y) + 
               ", 'theta': " + str(theta) + 
               ", 'name': '" + turtle_name + "'}\""],
        output = "both",
        shell = True
        )
    # 2. 调用客户端发送目标点位置
    client = Node(package = "cpp07_exercise", 
                  executable = "exer05_action_client", 
                  arguments = [str(x), str(y), str(theta)])
                # ros2 run cpp07_exercise exer05_action_client 6 9 0.0 --ros-args
    return LaunchDescription([spawn,client])
