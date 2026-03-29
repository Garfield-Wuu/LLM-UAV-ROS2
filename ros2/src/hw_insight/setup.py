import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'hw_insight'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hw',
    maintainer_email='toplaya@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard = hw_insight.offboard:main',
            'decode_px4_fmu_out_vehicle_status = hw_insight.msg_px4_fmu_out_vehicle_status:main',
            'keyboard_position = hw_insight.keyboard_position:main',
            'move_position = hw_insight.move_position:main',
            'keyboard_velocity = hw_insight.keyboard_velocity:main',
            'move_velocity = hw_insight.move_velocity:main',
            'text_command_bridge = hw_insight.text_command_bridge:main',
            'planner_velocity_bridge = hw_insight.planner_velocity_bridge:main',
            'ego_bspline_to_twist_relay = hw_insight.ego_bspline_to_twist_relay:main',
            'test_planner_feedback = hw_insight.test_planner_feedback:main',
            'gcs_dashboard = hw_insight.gcs_dashboard:main',
            'flight_regression_runner = hw_insight.flight_regression_runner:main',
            'llm_client = hw_insight.llm_client:main',
            'yolo_world_detector = hw_insight.yolo_world_detector:main',
            'target_grounding_node = hw_insight.target_grounding_node:main',
            'semantic_target_tf_node = hw_insight.semantic_target_tf_node:main',
            'semantic_goal_to_planner = hw_insight.semantic_goal_to_planner:main',
        ],
    },
)
