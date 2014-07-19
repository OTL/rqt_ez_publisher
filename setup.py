from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_ez_publisher', 'rqt_ez_publisher.widget',
              'rqt_ez_publisher.publisher', 'rqt_ez_publisher.quaternion_module'],
    package_dir={'': 'src'},
)

setup(**d)
