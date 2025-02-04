from setuptools import setup

package_name = 'ugv_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'mujoco',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='Caio Lorenzo Iriarte Salles',
    maintainer_email='caio.vaz137@gmail.com',
    description='Paquete para integrar MuJoCo con ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_pub = ugv_pkg.lidar:main',
            'camera_pub = ugv_pkg.camera:main',
            'init_nodes = ugv_pkg.node_init:main',
            'mujoco_pub = ugv_pkg.mujoco_pub:main'
        ],
    },
)

