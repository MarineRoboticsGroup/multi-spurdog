from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['spurdog_acomms_utils', 'leader'],
    package_dir={'': 'src'}
)
setup(**d)
