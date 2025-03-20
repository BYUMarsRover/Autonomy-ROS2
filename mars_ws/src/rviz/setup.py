from setuptools import find_packages, setup

package_name = 'rviz'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', ['launch/rviz_launch.py']),

    ],
    install_requires=['setuptools', 'ros2pkg', 'visualization_msgs'],
    zip_safe=True,
    maintainer='Avary Fielding',
    maintainer_email='avaryef@byu.edu',
    description='Launches Rviz',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz = rviz.rviz:main',
            'rviz_launch = launch.rviz_launch:main'
        ],
    },
)
