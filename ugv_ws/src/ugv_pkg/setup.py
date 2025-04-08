from setuptools import setup

package_name = 'ugv_pkg'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', ['ugv_pkg/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/robot_world_map.pgm',
    'worlds/robot_world_map.yaml',
    'worlds/robot_world.wbproj',
    'worlds/robot_world.wbt',
    'resource/ugv_robot.urdf',
]))

#   Append robot definition
data_files.append(('share/' + package_name + '/resource/protos/ugv_robot', [
    'ugv_robot/urdf/ugv_robot.proto'
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/ugv_robot.urdf'
]))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files
    ,
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'launch',
    ],
    zip_safe=True,
    maintainer='Caio Lorenzo Iriarte Salles',
    maintainer_email='caio.vaz137@gmail.com',
    description='Package for integration Webots - ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver = ugv_pkg.robot_driver:main',
        ],
        'launch.frontend.launch_extension': [
            'launch_ros = launch_ros',
        ],
    },
)

