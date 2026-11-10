import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Deklarasi argumen launch (bisa diubah saat menjalankan terminal)
    # Default: lintasan_mode = 0 (Lintasan A)
    lintasan_mode_arg = DeclareLaunchArgument(
        'lintasan_mode',
        default_value='0',
        description='Mode Lintasan: 0 untuk Lintasan A (Normal), 1 untuk Lintasan B'
    )

    # Mengambil nilai parameter dari argumen launch
    lintasan_mode_conf = LaunchConfiguration('lintasan_mode')

    # 2. Definisi Node 1: Vision Node
    vision_node = Node(
        package='asv26',
        executable='vision_node',  # Pastikan sesuai dengan entry_points di setup.py
        name='vision_node',
        output='screen',
        parameters=[{
            'lintasan_mode': lintasan_mode_conf
        }]
    )

    # 3. Definisi Node 2: Logic Node
    logic_node = Node(
        package='asv26',
        executable='logic_node',   # Pastikan sesuai dengan entry_points di setup.py
        name='logic_node',
        output='screen',
        parameters=[{
            'lintasan_mode': lintasan_mode_conf
        }]
    )

    # 4. Definisi Node 3: Control Node / MAVROS Bridge
    control_node = Node(
        package='asv26',
        executable='control_node', # Pastikan sesuai dengan entry_points di setup.py
        name='control_node',
        output='screen'
    )

    # 5. Definisi Node 4: Upload Node (Supabase / Telemetry)
    uploadA_node = Node(
        package='asv26',           
        executable='uploadA_node',  # Pastikan sesuai dengan entry_points di setup.py
        name='uploadA_node',
        output='screen',
        parameters=[{
            'lintasan_mode': lintasan_mode_conf
        }]
    )

    # 5. Definisi Node 4: Upload Node (Supabase / Telemetry)
    uploadB_node = Node(
        package='asv26',           
        executable='uploadB_node',  # Pastikan sesuai dengan entry_points di setup.py
        name='uploadB_node',
        output='screen',
        parameters=[{
            'lintasan_mode': lintasan_mode_conf
        }]
    )

    # 6. Gabungkan semua ke dalam Launch Description
    return LaunchDescription([
        lintasan_mode_arg,
        vision_node,
        logic1_node,
        control_node,
        uploadA_node,
        uploadB_node
    ])