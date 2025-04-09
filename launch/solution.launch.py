import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
#from launch.actions import IncludeLaunchdescription, TimerAction
#from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')
    #simulator_launch = os.path.join(get_package_share_directory('mpc_rbt_simulator'),'launch','simulation.launch.py') 

    return LaunchDescription([

    	# Spustí RViz
    	Node(
    	    package='rviz2',  # balíček pro RViz v ROS 2
    	    executable='rviz2',  # Spustí RViz
    	    name='rviz',
    	    output='screen',
			arguments=['-d', rviz_config_path]  # Použije konfigurační soubor rviz
        
    	),
    
#    	TimerAction(
#    		period = 3.0,
#    		actions = [
#    			IncludeLaunchDescription(
#    		PythonLaunchDescriptionSource(simulator_launch)
#    		)
#    		]
#    
#    	),
    
    	#TimerAction(
    	#	period = 10.0,
    	#	actions = [
    			# Spustí tvoji nody
    	Node(
     	   	package='mpc_rbt_student',  # Název balíčku, kde je tvoje nody
        	executable='localization',  # Název souboru, který obsahuje tvoji nody
        	name='localization_node',  # Název pro tuto nody (můžeš zvolit jakýkoliv název)
        	output='screen'
    	)
    	#		]	
    	#)
    ])
