from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  packages=['camera_integrators', 'behaviors', 'helpers'],
  package_dir={'': 'src'}
)

setup(**d)