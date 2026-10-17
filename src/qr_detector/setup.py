from setuptools import find_packages, setup

package_name = 'qr_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/qr_detector.yaml']),
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
            'qr_detector_node = qr_detector.qr_detector_node:main',
            'move_forward = qr_detector.move_forward:main',
        ],
    },
)
