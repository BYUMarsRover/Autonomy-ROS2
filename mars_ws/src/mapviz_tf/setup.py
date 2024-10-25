from setuptools import find_packages, setup

package_name = 'mapviz_tf'

setup(
    name= package_name,
    version="0.0.0",
    packages =find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Crandall',
    maintainer_email="wyomike2020@gmail.com",
    description='The MapViz package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'click_waypoint = click_waypoint:main'  #Figure out what we'll put in here instead of the node name
            'gps_to_mapviz = gps_to_mapviz:main'
            'path_to_mapviz = path_to_mapviz:main'
            'rover_tf_broadcast = rover_tf_broadcast:main'
        ],
    },
)