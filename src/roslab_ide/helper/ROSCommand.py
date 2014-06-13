__author__ = 'privat'

import rosnode
import rosgraph
from roslab_ide.widgets.ExternalProcessWidget import ExternalProcessWidget


class ROSCommand():

    def __init__(self):
        pass

    @staticmethod
    def roscore():
        # check if roscore is running
        """
        Start ROS master.

        :return:
        """
        if rosgraph.is_master_online():
            print('roscore already started!')
            return
        ROSCommand.execute('roscore')

    @staticmethod
    def rviz(config=None):
        # check if roscore is running
        """
        Start rviz, the standard ROS visualization tool.

        :param config:
        :return:
        """
        if not rosgraph.is_master_online():
            print('roscore is not online! Start roscore before running any other node including rviz.')
            return
        for node in rosnode.get_node_names():
            if node.find('rviz') != -1:
                print('rviz already started!')
                return
        if config:
            command = 'rosrun rviz rviz -d {}'.format(config)
        else:
            command = 'rosrun rviz rviz'
        ROSCommand.execute(command)

    @staticmethod
    def rqt():
        """
        Start rqt, the standard ROS GUI plugin loader.

        Note: rqt itself does not need rosmaster. Its plugins will need the rosmaster, of course!
        """
        ROSCommand.execute('rqt')

    @staticmethod
    def rosrun(package, node, args=None):
        """
        Start ROS node from given package.

        :param package: Package name
        :param node: Node name
        :param args: additional command line arguments
        """
        if args:
            command = 'rosrun {0} {1}_node {2}'.format(package, node, args)
        else:
            command = 'rosrun {0} {1}_node'.format(package, node)
        ROSCommand.execute(command)

    @staticmethod
    def roslaunch(package, launch_file):
        """
        Start ROS launch file from package.

        :param package:
        :param launch_file:
        """
        command = 'roslaunch {} {}.launch'.format(package, launch_file)
        ROSCommand.execute(command, keep_open=True)

    @staticmethod
    def wstool(command, name, vcs, uri, version):
        """
        Start ROS workspace tool to manage rosinstall data.

        :param command:
        :param name:
        :param vcs:
        :param uri:
        :param version:
        """
        command = 'wstool {0} {1} --{2} {3} --version={4}'.format(command, name, vcs, uri, version)
        ROSCommand.execute(command)

    @staticmethod
    def catkin_make(working_dir, package=None):
        """
        Start catkin_make to build the whole workspace or the given package.

        :param working_dir: Working directory of external process
        :param package: Package to build
        """
        if package:
            command = 'catkin_make {}'.format(package)
        else:
            command = 'catkin_make'
        ROSCommand.execute(command, working_dir)

    @staticmethod
    def execute(command, working_dir=None, keep_open=False):
        """
        Open a widget and execute given command in external process.

        :param command: Command to execute
        :param working_dir: Working directory of external process
        :return:
        """
        print('executing "{}" in external terminal...'.format(command))
        if working_dir:
            process_widget = ExternalProcessWidget(command, working_dir, keep_open=keep_open)
        else:
            process_widget = ExternalProcessWidget(command, keep_open=keep_open)
        process_widget.show()
        # return new process
        return process_widget
