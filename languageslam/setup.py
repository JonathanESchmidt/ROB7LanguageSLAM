from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['languageslam'],
    scripts=['bin/map_converter'],
    package_dir={'': 'src'}
)

setup(**d)
