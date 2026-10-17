from glob import glob

from setuptools import find_packages, setup

package_name = 'mission_management'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files live in src/launch/; installing them here allows
        # `ros2 launch mission_management mission.launch.py` / `sitl.launch.py`.
        ('share/' + package_name + '/launch', glob('../launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='satyam',
    maintainer_email='skiitm198@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_management_node = mission_management.mission_management_node:main',
            'mission_tree = mission_management.mission_tree:main',
        ],
    },
)
