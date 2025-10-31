import os,launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ============================================================================
    #全局参数
    package_name = 'ros2_livox_simulation'
    robot_name = 'my_robot'
    world_file_name = 'bigHHH.world'
    use_sim_time = True
    gui = True
    # ============================================================================


    # ============================================================================
    #bash传参
    use_sim_time_arg = LaunchConfiguration('use_sim_time', default=use_sim_time)
    gui_arg = LaunchConfiguration('gui', default=gui)
    # ============================================================================

    # ============================================================================
    #包路径
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ros2_livox_simulation = get_package_share_directory(package_name)
    pkg_share = get_package_share_directory(package_name)[:-len(package_name)]  #gazebo模型查找路径
    # ============================================================================

    # ============================================================================
    #给GAZEBO_MODEL_PATH添加 mid360.stl的路径
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share
    # ============================================================================
    
    
    # ============================================================================
    #gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        #empty   standardrobots_factory
        launch_arguments={
            'world': os.path.join(pkg_ros2_livox_simulation, 'worlds', world_file_name),
            'gui': gui_arg,
            'verbose': 'true'
        }.items()
    )
    
    #robot_state_publisher
    xacro_file = os.path.join(pkg_ros2_livox_simulation, 'urdf',robot_name, robot_name +'.xacro')
    robot_description = Command([f'xacro {xacro_file}'])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # joint_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    #spawn_entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mid360_platform', '-topic', 'robot_description'],
        output='screen'
    )

    # rviz
    rviz_path = os.path.join(pkg_ros2_livox_simulation, 'rviz', robot_name + '.rviz')
    rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d',rviz_path],
    )

    #load_joint_state_controller
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'myrobot_joint_state_broadcaster'],
        output='screen'
    )
    # ============================================================================

    # ============================================================================
    #机器人加载完毕后执行加载控制器 事件处理
    load_controller_event = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller]
        )
    )
    # ============================================================================



    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(rviz) 
    ld.add_action(load_controller_event)
    return ld

