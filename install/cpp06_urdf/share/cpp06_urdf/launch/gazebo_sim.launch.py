import launch
import launch_ros
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_packages_path = get_package_share_directory('cpp06_urdf')
    default_xacro_path = os.path.join(urdf_packages_path, 'urdf', 'xacro', 'car.urdf.xacro')
    default_gazebo_world_path = os.path.join(urdf_packages_path, 'worlds', 'custorm_room.world')

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
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(urdf_packages_path, 'urdf') + ':' + os.path.join(urdf_packages_path, 'models')

    # 生成机器人描述
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str
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

    # 生成 spawn_entity 节点
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            LaunchConfiguration('entity_name'),  # 动态实体名称
            '-topic',
            TextSubstitution(text='/robot_description')
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        declare_model_arg,
        declare_entity_name_arg,
        robot_state_publisher,
        launch_gazebo,
        spawn_entity
    ])