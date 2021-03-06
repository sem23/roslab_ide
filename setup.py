## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['roslab_ide', 'roslab_ide.helper', 'roslab_ide.generators', 'roslab_ide.widgets', 'roslab_ide.dialogs'],
    package_dir={
        'roslab_ide': 'src',
        'roslab_ide.helper': 'src',
        'roslab_ide.generators': 'src',
        'roslab_ide.widgets': 'src',
        'roslab_ide.dialogs': 'src'},
    requires=['rospy', 'rospkg', 'rosmsg', 'yaml', 'pygraphviz', 'PyQt4']
)

setup(**setup_args) 
