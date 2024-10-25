from distutils.core import setup

package_name = 'mapviz_tf'

setup(
    name= package_name,
    version="0.0.0",
    install_requires=['setuptools'],
    zip_safe=True,
    packages = ['mapviz_tf'],
    package_dir={'': 'src'},
    maintainer='Michael Crandall',
    maintainer_email="wyomike2020@gmail.com",
    description='The MapViz package',
    license='TODO',

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name])
        ('share/' + package_name, ['package.xml'])
    ],
    entry_points={
        'console_scripts': [
            'click_waypoint = click_waypoint:main'  #Figure out what we'll put in here instead of the node name
            'gps_to_mapviz = gps_to_mapviz:main'
            'path_to_mapviz = path_to_mapviz:main'
            'rover_tf_broadcast = rover_tf_broadcast:main'
        ]
    }
)