from setuptools import find_packages, setup
import os
import glob

package_name = 'mapviz_tf'

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/params', glob.glob('params/*.yaml')),
        ('share/' + package_name + '/params', glob.glob('scripts/*.py')),
        ('share/' + package_name + '/scripts', glob.glob('scripts/.mapviz_config')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
        ('share/' + package_name + '/image', glob.glob('image/*.png')),
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
            'click_waypoint = mapviz_tf.click_waypoint:main',  # Separate each entry with a comma
            'gps_to_mapviz = mapviz_tf.gps_to_mapviz:main',
            'path_to_mapviz = mapviz_tf.path_to_mapviz:main',
            'rover_tf_broadcaster = mapviz_tf.rover_tf_broadcaster:main',
            'path_test = mapviz_tf.path_broadcaster_test:main',
            'wgs84_tf = mapviz_tf.wgs84_tf_broadcaster:main',
        ],
    },
)
