import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'science'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=find_packages(exclude=['test']),
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Include the gui .ui file
        (os.path.join('share', package_name, 'gui'), glob(os.path.join('science', 'gui', '*.ui'))),
        #include resources
        (os.path.join('share', package_name, 'gui'), glob(os.path.join('science', 'presentation', '*.txt'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sarah Sanderson',
    maintainer_email='sarah.g.sanderson@gmail.com',
    classifiers=[
        'Intended Audience :: Mars Rover Team',
        'License :: MIT',
        'Programming Language :: Python',
        'Topic :: Science',
    ],
    description='Package the runs the science module from the rover or from the base station.',
    license='MIT',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'science_control = science.science_control:main',
            'science_serial_interface = science.science_serial_interface:main',
            'science_data_saver = science.presentation.science_data_saver:main',
            'science_GUI = science.gui.science_GUI:main',
            # 'presentation_generator = science.presentation.presentation_generator:main'
        ],
    },
)