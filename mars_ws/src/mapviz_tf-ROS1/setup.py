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
)
