from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['flyingros_pose'],
    package_dir={'': 'src'})

setup(**setup_args)
