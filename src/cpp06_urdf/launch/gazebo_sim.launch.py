import launch
import launch_ros
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_packages_path = get_package_share_directory("cpp06_urdf")
    default_xacro_path = os.path.join(urdf_packages_path, 'urdf', 'xacro', 'car.urdf.xacro')
    default_gazebo_world_path = os.path.join(urdf_packages_path, 'worlds', 'custom_room.world')  # 确保文件名正确

    # 声明参数
    declare_model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_xacro_path,
        description='Xacro/URDF 模型文件路径'
    )
    declare_entity_name_arg = DeclareLaunchArgument(
        name='entity_name',
        default_value='my_robot',
        description='Gazebo 实体名称'
    )

    # 设置 Gazebo 模型路径
    gazebo_model_path = os.path.join(urdf_packages_path, 'urdf') + ':' + \
                        os.path.join(urdf_packages_path, 'models') + ':' + \
                        os.environ.get('GAZEBO_MODEL_PATH', '')
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    # 生成机器人描述
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # 启动 robot_state_publisher
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )

    # 启动 Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': default_gazebo_world_path}.items()
    )

    # 生成 spawn_entity 节点（直接使用 TimerAction 延迟）
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', LaunchConfiguration('entity_name'),
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # 延迟 5 秒启动 spawn_entity
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    return launch.LaunchDescription([
        declare_model_arg,
        declare_entity_name_arg,
        robot_state_publisher,
        launch_gazebo,
        delayed_spawn  # 直接添加延迟动作，无需事件处理器
    ])