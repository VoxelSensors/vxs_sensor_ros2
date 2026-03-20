import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # ==========================================
    # 1. SET YOUR PATHS HERE
    # ==========================================
    bag_path = '../../../../../bags/lab_wire_1.5cm_1m_medium/lab_wire_1.5cm_1m_medium_0.db3' # Point this to your actual bag!
    script_dir = '../scripts/'

    # ==========================================
    # 2. PLAY THE BAG
    # ==========================================
    # --clock is crucial here so RViz and your nodes sync perfectly with the recorded time
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
        output='screen'
    )

    # ==========================================
    # 3. RUN FAKE ODOMETRY
    # ==========================================
    run_fake_odom = ExecuteProcess(
        cmd=[
            'python3', 'fake_odometry.py',
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'v_x:=0.45',       # Tune your speed here
            '-p', 't_start:=2.5',   # Tune start time here
            '-p', 't_stop:=9.0'     # Tune stop time here
        ],
        cwd=script_dir,  # This tells the launch file where to find your python script
        output='screen'
    )

    # ==========================================
    # 4. RUN VOXELISATION
    # ==========================================
    run_voxelisation = ExecuteProcess(
        cmd=[
            'python3', 'voxel_decimator.py',
            '--ros-args',
            '-p', 'use_sim_time:=true',
        ],
        cwd=script_dir,  # This tells the launch file where to find your python script
        output='screen'
    )

    return LaunchDescription([
        play_bag,
        run_fake_odom,
        run_voxelisation
    ])