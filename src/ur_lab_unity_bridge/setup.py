from setuptools import setup

package_name = 'ur_lab_unity_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Gazebo-API emulation for running ur_lab nodes against Unity',
    license='Apache License 2.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/unity_sim.launch.py']),
    ],
    entry_points={
        'console_scripts': [
            'unity_sim = ur_lab_unity_bridge.unity_sim:main',
        ],
    },
)
