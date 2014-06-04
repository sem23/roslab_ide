__author__ = 'privat'

from roslab_ide.widgets.ExternalProcessWidget import ExternalProcessWidget


class ROSCommand():

    def __init__(self):
        pass

    @staticmethod
    def roscore():
        command = 'roscore'
        ROSCommand.execute(command)

    @staticmethod
    def rviz(config=None):
        if config:
            command = 'rosrun rviz rviz -d {}'.format(config)
        else:
            command = 'rosrun rviz rviz'
        ROSCommand.execute(command)

    @staticmethod
    def rqt():
        ROSCommand.execute('rqt')

    @staticmethod
    def rosrun(package, node, args=None):
        if args:
            command = 'rosrun {0} {1}_node {2}'.format(package, node, args)
        else:
            command = 'rosrun {0} {1}_node'.format(package, node)
        ROSCommand.execute(command)

    @staticmethod
    def wstool(command, name, vcs, uri, version):
        command = 'wstool {0} {1} --{2} {3} --version={4}'.format(command, name, vcs, uri, version)
        ROSCommand.execute(command)

    @staticmethod
    def catkin_make(working_dir, package=None):
        if package:
            command = 'catkin_make {}'.format(package)
        else:
            command = 'catkin_make'
        ROSCommand.execute(command, working_dir)

    @staticmethod
    def execute(command, working_dir=None):
        print('executing "{}" in external terminal...'.format(command))
        if working_dir:
            process_widget = ExternalProcessWidget(command, working_dir)
        else:
            process_widget = ExternalProcessWidget(command)
        process_widget.show()
