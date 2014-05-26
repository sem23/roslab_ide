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
    def rosrun(package, node):
        command = 'rosrun {0} {1}_node'.format(package, node)
        ROSCommand.execute(command)

    @staticmethod
    def wstool(command, name, vcs, uri, version):
        command = 'wstool {0} {1} --{2} {3} --version={4}'.format(command, name, vcs, uri, version)
        ROSCommand.execute(command)

    @staticmethod
    def catkin_make(working_dir):
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
