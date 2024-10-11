from setuptools import setup
import os
import glob

package_name = 'navigation'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Braden Meyers',
 maintainer_email='bjm255',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'obstacle_detect_node = navigation.obstacle_detect_node:main'
     ],
   },
)