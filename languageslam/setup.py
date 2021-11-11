from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['languageslam'],
    package_dir={'': 'src'}
    requires=['std_msgs', 'rospy', 'nav_msgs']
)
setup(**d)
