from vrp_gz.bridge import Bridge, BridgeDirection

def pose(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose',
        ros_topic='pose',
        gz_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)

def pose_static(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose_static',
        ros_topic='pose_static',
        gz_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)

# FIXME: исправить название топиков
def thrust(model_name, side):
    return Bridge(
        gz_topic=f'{model_name}/thrusters/{side}/thrust',
        ros_topic=f'thrusters/{side}/thrust',
        gz_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)


def clock():
    return Bridge(
        gz_topic='/clock',
        ros_topic='/clock',
        gz_type='ignition.msgs.Clock',
        ros_type='rosgraph_msgs/msg/Clock',
        direction=BridgeDirection.GZ_TO_ROS)

