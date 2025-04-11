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
        ('share/' + package_name + '/presentation/resources', 
            glob('science/presentation/resources/*')),  # Include your resource files
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include the gui .ui file
        (os.path.join('share', package_name, 'gui'), glob(os.path.join('science', 'gui', '*.ui'))),
        # Include the debug_gui .ui files
        (os.path.join('share', package_name, 'debug_gui'), glob(os.path.join('science', 'debug_gui', '*.ui'))),
        
        # Include the rxtx_gui .csv files
        (os.path.join('share', package_name, 'rxtx_gui'), glob(os.path.join('science', 'rxtx_gui', '*.ui'))),
        # Include the rxtx_gui .csv files
        (os.path.join('share', package_name, 'response_gui'), glob(os.path.join('science', 'response_gui', '*.ui'))),

        (os.path.join('share', package_name, 'function_mapping'), glob(os.path.join('science', 'function_mapping', '*.csv'))),
        #Might want to be more particular in the future but this is a quick fix to get everything
        # (os.path.join('share', package_name), glob(os.path.join('science', '*.py'))),
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
            'science_serial_interface = science.science_serial_interface:main',
            'science_request_manager = science.science_request_manager:main',
            'science_data_saver = science.presentation.science_data_saver:main',
            'science_GUI = science.gui.science_GUI:main',
            'science_debug = science.debug_gui.science_debug_GUI:main',
            'science_rxtx = science.rxtx_gui.science_rxtx_GUI:main',
            'science_response = science.response_gui.response_GUI:main'
            # 'presentation_generator = science.presentation.presentation_generator:main'
        ],
    },
)