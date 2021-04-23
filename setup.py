from setuptools import setup

package_name = 'ds_ros2_use'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='martinmaealand',
    maintainer_email='martinmaeland@outlook.com',
    description='ROS2 node for transfering data between ground station and drone.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'use_transfer = ds_ros2_use.use_transfer:main'
        ],
    },
)
