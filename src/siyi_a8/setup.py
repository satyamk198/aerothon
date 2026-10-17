from setuptools import find_packages, setup

package_name = 'siyi_a8'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='satyam',
    maintainer_email='skiitm198@gmail.com',
    description='SIYI A8 mini on Ethernet: RTSP video to /camera/image_raw and gimbal pointing via the SIYI SDK UDP protocol',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtsp_camera_node = siyi_a8.rtsp_camera_node:main',
            'gimbal_node = siyi_a8.gimbal_node:main',
            'photo_node = siyi_a8.photo_node:main',
        ],
    },
)
