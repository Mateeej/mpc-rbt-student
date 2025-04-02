import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    # Spustí tvoji nody
    localization_node = Node(
        package='mpc_rbt_student',  # Název balíčku, kde je tvoje nody
        executable='localization',  # Název souboru, který obsahuje tvoji nody
        name='localization_node',  # Název pro tuto nody (můžeš zvolit jakýkoliv název)
        output='screen'
    )

    # Spustí RViz
    rviz_node = Node(
        package='rviz2',  # balíček pro RViz v ROS 2
        executable='rviz2',  # Spustí RViz
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_path]  # Použije konfigurační soubor rviz
    )
    
    return LaunchDescription([
	localization_node,
	rviz_node
    ])
