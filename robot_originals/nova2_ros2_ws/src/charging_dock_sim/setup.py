from setuptools import setup

package_name = 'charging_dock_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/charging_dock_demo.launch.py', 'launch/auto_dock_demo.launch.py']),
        ('share/' + package_name + '/config', ['config/docking_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'scene_publisher = charging_dock_sim.scene_publisher:main',
            'auto_dock = charging_dock_sim.auto_dock:main',
            'plan_docking = charging_dock_sim.plan_docking:main',
        ],
    },
)
