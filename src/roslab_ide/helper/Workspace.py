__author__ = 'privat'

import os
import re
import stat
import yaml
import time
import pygraphviz

import rospkg
rp = rospkg.RosPack()

# pyqt imports
from PyQt4.QtCore import pyqtSlot, QSettings
from PyQt4.QtGui import QAction, QMenu, QTreeWidgetItem, QIcon, QLineEdit
from PyQt4.QtGui import QDialog, QInputDialog, QFileDialog, QTreeWidgetItem, QMessageBox

import roslab_ide.helper.globals as g

from roslab_ide.dialogs.RosinstallPackageDialog import RosinstallPackageDialog
from roslab_ide.dialogs.ArgumentDialog import ArgumentDialog
from roslab_ide.dialogs.RosLaunchNodeDialog import RosLaunchNodeDialog
from roslab_ide.dialogs.ImportDialog import ImportDialog
from roslab_ide.dialogs.ParameterDialog import ParameterDialog
from roslab_ide.dialogs.CommunicationDialog import CommunicationDialog

from roslab_ide.helper.ROSCommand import ROSCommand
from roslab_ide.backends.PyBackend import PyBackend
from roslab_ide.generators.CMakeListsTxtGenerator import CMakeListsTxtGenerator
from roslab_ide.generators.PackageXmlGenerator import PackageXmlGenerator
from roslab_ide.generators.SetupPyGenerator import SetupPyGenerator
from roslab_ide.generators.RosLaunchFileGenerator import RosLaunchFileGenerator
from roslab_ide.IDE import __version__


class TreeItem(QTreeWidgetItem):

    def __init__(self, parent, type=g.UNDEFINED_ITEM, key=None, value=None, data=None):
        QTreeWidgetItem.__init__(self, parent, type)
        # set value as default item name
        self._name = value
        # set key as first column text
        if key:
            self.setText(0, key)
        # set value as seconds column text
        if value:
            self.setText(1, str(value))
        # add data as key value items
        if data:
            # add key value items from data
            self.add_key_value_items(data=data)
            # overwrite item name and value if data has key 'name'
            if 'name' in data:
                self._name = data['name']
                self.setText(1, self._name)
        # setup item actions containers
        self._mod_actions = []
        self._add_actions = []
        # cache context menu
        self._context_menu = None

    def __lt__(self, other):
        return self.type() < other.type()

    def add_key_value_item(self, data, key, value):
        """
        Add a single key value item to this item and return it.
        """
        return KeyValueItem(parent=self, data=data, key=key, value=value)

    def add_key_value_items(self, data):
        """
        Add dictionaries key value pairs as items to this item and return them as list.
        """
        key_value_items = []
        for key, value in data.iteritems():
            # only add simple types as key value items
            if type(value) is list or type(value) is dict:
                continue
            key_value_items.append(KeyValueItem(parent=self, data=data, key=key, value=value))
        return key_value_items

    def get_context_menu(self):
        """
        Build context menu for item depending on superclass actions and return it.
        """
        # return None if item has no actions
        if not (len(self._mod_actions) or len(self._add_actions)):
            return None
        # return cached context menu
        if self._context_menu:
            return self._context_menu
        context_menu = QMenu(self._name)
        context_menu.addActions(self._mod_actions)
        if len(self._add_actions):
            add_menu = context_menu.addMenu('Add')
            add_menu.setIcon(QIcon(os.path.join(
                rp.get_path('roslab_ide'), 'resource', 'icons', 'plus.png')))
            add_menu.addActions(self._add_actions)
        # cache context menu
        self._context_menu = context_menu
        # return freshly build context menu
        return context_menu

    def get_actions(self):
        return self._mod_actions + self._add_actions

    def add_action(self, name, icon, slot, add=False):
        new_action = QAction(name, None)
        new_action.setIcon(QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', icon)))
        new_action.triggered.connect(slot)
        if add:
            self._add_actions.append(new_action)
        else:
            self._mod_actions.append(new_action)

    def setWTS(self, key_info, value_info):
        self.setWhatsThis(0, key_info)
        self.setWhatsThis(1, value_info)
        self.setToolTip(0, key_info)
        self.setToolTip(1, value_info)
        self.setStatusTip(0, key_info)
        self.setStatusTip(1, value_info)

    def name(self):
        return self._name

    def update(self):
        pass


class WorkspaceItem(TreeItem):

    def __init__(self, parent):
        TreeItem.__init__(self, parent=parent, type=g.WORKSPACE_ITEM)
        # set name
        self._name = 'workspace'
        # set info
        self.setWTS('Current workspace', 'Workspace path')

        # setup mod actions
        self.add_action('change', 'change.png', self.change_workspace)
        self.add_action('build', 'build.png', self.build_workspace)

        # setup add actions
        self.add_action('ROSLab package', 'open-box.png', self.add_roslab_package, add=True)
        self.add_action('rosinstall package', 'box.png', self.add_rosinstall_package, add=True)

    @pyqtSlot()
    def change_workspace(self):
        Controller.change_workspace()

    @pyqtSlot()
    def build_workspace(self):
        Controller.build_workspace()

    @pyqtSlot()
    def add_roslab_package(self):
        Controller.add_roslab_package()

    @pyqtSlot()
    def add_rosinstall_package(self):
        Controller.add_rosinstall_package()


class RoslabPackagesItem(TreeItem):

    def __init__(self, parent):
        TreeItem.__init__(self, parent=parent, type=g.ROSLAB_ITEM, key='ROSLab')
        # set name
        self._name = 'ROSLab'
        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'open-boxs.png')))
        # set info
        self.setWTS('ROSLab packages', 'ROSLab packages count')

        self._workspace_item = parent

        # mod actions
        # add actions
        self.add_action('Package', 'open-box.png', self._workspace_item.add_roslab_package, add=True)

        # vars
        self._package_count = 0

    def add_package_item(self, data):
        # update package count
        self._package_count += 1
        self.setText(1, str(self._package_count))
        return RoslabPackageItem(parent=self, data=data)

    def remove_package_item(self):
        # update package count
        self._package_count -= 1
        self.setText(1, str(self._package_count))
        # TODO: implement me!
        pass


class RoslabPackageItem(TreeItem):

    def __init__(self, parent, data=None):
        TreeItem.__init__(self, parent=parent, type=g.ROSLAB_PACKAGE_ITEM, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'open-box.png')))
        # set info
        self.setWTS('ROSLab package', 'ROSLab package name')

        # mod actions
        self.add_action('generate', 'build.png', self.generate)
        self.add_action('build', 'build.png', self.build)

        # add actions
        self.add_action('Dependency', 'dependency.png', self.add_dependency, add=True)
        self.add_action('Library', 'binary.png', self.add_library, add=True)
        self.add_action('Node', 'executable-script.png', self.add_node, add=True)
        self.add_action('ROS launch', 'ros_launch.png', self.add_ros_launch_file, add=True)
        self.add_action('Robot Concert launch', 'rocon_launch.png', self.add_rocon_launch_file, add=True)

        # group items
        self._dependencies_item = None
        self._libraries_item = None
        self._nodes_item = None
        self._ros_launch_files_item = None

        if data:
            # set package name as item value
            self._name = data['name']
            self.setText(1, self._name)
            # add child items
            if 'dependencies' in data:
                self._dependencies_item = TreeItem(parent=self)
                self._dependencies_item.setText(0, 'Dependencies')
                self._dependencies_item.setIcon(0, QIcon(
                    os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'dependencies.png')))
                for dependency in data['dependencies']:
                    self.add_dependency_item(data=dependency)
            if 'libraries' in data:
                self._libraries_item = TreeItem(parent=self)
                self._libraries_item.setText(0, 'Libraries')
                self._libraries_item.setIcon(0, QIcon(
                    os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'libraries.png')))
                for library in data['libraries']:
                    self.add_library_item(data=library)
            if 'nodes' in data:
                self._nodes_item = TreeItem(parent=self)
                self._nodes_item.setText(0, 'Nodes')
                self._nodes_item.setIcon(0, QIcon(
                    os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'nodes.png')))
                for node in data['nodes']:
                    self.add_node_item(data=node)
            if 'ros_launch' in data:
                self._ros_launch_files_item = TreeItem(parent=self)
                self._ros_launch_files_item.setText(0, 'ROS launch files')
                self._ros_launch_files_item.setIcon(0, QIcon(
                    os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'ros_launchs.png')))
                for launch_file in data['ros_launch']:
                    self.add_ros_launch_file_item(data=launch_file)

    def add_dependency_item(self, data):
        if not self._dependencies_item:
            self._dependencies_item = TreeItem(parent=self)
            self._dependencies_item.setText(0, 'Dependencies')
            self._dependencies_item.setIcon(0, QIcon(
                os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'dependencies.png')))
        item = DependencyItem(parent=self._dependencies_item, data=data, package_name=self._name)
        self._dependencies_item.setText(1, str(self._dependencies_item.childCount()))
        return item

    def add_library_item(self, data):
        if not self._libraries_item:
            self._libraries_item = TreeItem(parent=self)
            self._libraries_item.setText(0, 'Libraries')
            self._libraries_item.setIcon(0, QIcon(
                os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'libraries.png')))
        item = LibraryItem(parent=self._libraries_item, data=data, package_name=self._name)
        self._libraries_item.setText(1, str(self._libraries_item.childCount()))
        return item

    def add_node_item(self, data):
        if not self._nodes_item:
            self._nodes_item = TreeItem(parent=self)
            self._nodes_item.setText(0, 'Nodes')
            self._nodes_item.setIcon(0, QIcon(
                os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'nodes.png')))
        item = NodeItem(parent=self._nodes_item, data=data, package_name=self._name)
        self._nodes_item.setText(1, str(self._nodes_item.childCount()))
        return item

    def add_ros_launch_file_item(self, data):
        if not self._ros_launch_files_item:
            self._ros_launch_files_item = TreeItem(parent=self)
            self._ros_launch_files_item.setText(0, 'ROS launch files')
            self._ros_launch_files_item.setIcon(0, QIcon(
                os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'ros_launchs.png')))
        item = RosLaunchFileItem(parent=self._ros_launch_files_item, data=data, package_name=self._name)
        self._ros_launch_files_item.setText(1, str(self._ros_launch_files_item.childCount()))
        return item

    @pyqtSlot()
    def generate(self):
        Controller.generate_package(package=self._name)

    @pyqtSlot()
    def build(self):
        # currently this seems to be senseless because python-backend packages need no 'build'
        Controller.build_package(package=self._name)

    @pyqtSlot()
    def add_dependency(self):
        data = Controller.add_dependency(self._name)
        if data:
            self.add_dependency_item(data=data)

    @pyqtSlot()
    def add_library(self):
        data = Controller.add_library(self._name)
        if data:
            self.add_library_item(data=data)

    @pyqtSlot()
    def add_node(self):
        data = Controller.add_node(self._name)
        if data:
            self.add_node_item(data=data)

    @pyqtSlot()
    def add_ros_launch_file(self):
        data = Controller.add_ros_launch_file(self._name)
        if data:
            self.add_ros_launch_file_item(data=data)

    @pyqtSlot()
    def add_rocon_launch_file(self):
        # TODO: implement me!
        pass


class RosinstallPackagesItem(TreeItem):

    def __init__(self, parent):
        TreeItem.__init__(self, parent=parent, type=g.ROSINSTALL_ITEM, key='rosinstall')

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'boxs.png')))
        # set info
        self.setWTS('rosinstall packages', 'rosinstall packages count')

        self._workspace_item = parent

        # mod actions
        # add actions
        add_package_action = QAction('Package', None)
        add_package_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'box.png')))
        add_package_action.triggered.connect(self._workspace_item.add_rosinstall_package)
        self._add_actions = [add_package_action]

        # vars
        self._package_count = 0

    def add_package_item(self, data):
        self._package_count += 1
        self.setText(1, str(self._package_count))
        return RosinstallPackageItem(parent=self, data=data)


class RosinstallPackageItem(TreeItem):

    def __init__(self, parent, data=None):
        TreeItem.__init__(self, parent=parent, type=g.ROSINSTALL_PACKAGE_ITEM, data=data)
        # set name
        self._name = 'rosinstall'
        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'box.png')))
        # set info
        self.setWTS('rosinstall package', 'rosinstall package name')


class DependencyItem(TreeItem):

    def __init__(self, parent, data=None, package_name=None):
        TreeItem.__init__(self, parent=parent, type=g.DEPENDENCY_ITEM, data=data)

        # set parent package name
        self._package_name = package_name

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'dependency.png')))
        # set info
        self.setWTS('Dependency', 'Dependency name')


class LibraryItem(TreeItem):

    def __init__(self, parent, data, package_name):
        TreeItem.__init__(self, parent=parent, type=g.LIBRARY_ITEM, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'binary.png')))
        # set info
        self.setWTS('Library', 'Library name')

        # set parent package name
        self._package_name = package_name
        self._comm_item = None
        self._machines_item = None
        self._functions_item = None
        self._tf_item = None

        # mod actions
        # add actions
        self.add_action('Import', 'import.png', self.add_import, add=True)
        self.add_action('Parameter', 'param.png', self.add_param, add=True)
        self.add_action('Basic Communication', 'communication.png', self.add_basic_comm, add=True)
        self.add_action('Tf Broadcaster', 'comm_out.png', self.add_tf_broadcaster, add=True)
        self.add_action('Tf Listener', 'comm_in.png', self.add_tf_listener, add=True)
        self.add_action('State Machine', 'state_machine.png', self.add_state_machine, add=True)
        self.add_action('Function', 'function.png', self.add_function, add=True)

        # setup data
        if data:
            # add items
            if 'import' in data:
                for entry in data['import']:
                    self.add_import_item(data=entry)
            if 'param' in data:
                for entry in data['param']:
                    self.add_parameter_item(data=entry)
            if 'comm' in data:
                self._comm_item = CommunicationsItem(parent=self)
                self._comm_item.setIcon(0, QIcon(
                    os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communications.png')))
                if 'pub' in data['comm']:
                    for entry in data['comm']['pub']:
                        self.add_publisher_item(data=entry)
                if 'sub' in data['comm']:
                    for entry in data['comm']['sub']:
                        self.add_subscriber_item(data=entry)
                if 'ss' in data['comm']:
                    for entry in data['comm']['ss']:
                        self.add_service_server_item(data=entry)
                if 'sc' in data['comm']:
                    for entry in data['comm']['sc']:
                        self.add_service_client_item(data=entry)
                if 'as' in data['comm']:
                    for entry in data['comm']['as']:
                        self.add_action_server_item(data=entry)
                if 'ac' in data['comm']:
                    for entry in data['comm']['ac']:
                        self.add_action_client_item(data=entry)
                if 'sas' in data['comm']:
                    for entry in data['comm']['sas']:
                        self.add_simple_action_server_item(data=entry)
                if 'sac' in data['comm']:
                    for entry in data['comm']['sac']:
                        self.add_simple_action_client_item(data=entry)
            if 'fsm' in data:
                self._machines_item = TreeItem(parent=self, type=g.STATE_MACHINES_ITEM)
                self._machines_item.setIcon(0, QIcon(os.path.join(
                    rp.get_path('roslab_ide'), 'resource', 'icons', 'state_machines.png')))
                for fsm in data['fsm']:
                    self.add_state_machine_item(data=fsm)
            if 'tf' in data:
                self._tf_item = TransformationsItem(parent=self)
                if 'broadcaster' in data['tf']:
                    for entry in data['tf']['broadcaster']:
                        self.add_tf_broadcaster_item(data=entry)
                if 'listener' in data['tf']:
                    for entry in data['tf']['listener']:
                        self.add_tf_listener_item(data=entry)
            if 'functions' in data:
                self._functions_item = FunctionsItem(parent=self)
                for entry in data['functions']:
                    self.add_function_item(data=entry)

    def add_import_item(self, data):
        item = TreeItem(parent=self, data=data)
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'import.png')))
        # set info
        item.setWTS('Import', 'Import')
        return item

    def add_parameter_item(self, data):
        item = TreeItem(parent=self, data=data)
        # set icon
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'param.png')))
        # set info
        item.setWTS('Parameter', 'Parameter name')
        return item

    def add_publisher_item(self, data):
        if not self._comm_item:
            self._comm_item = CommunicationsItem(parent=self)
        item = TreeItem(parent=self._comm_item, data=data)
        # update communications count
        self._comm_item.setText(1, str(self._comm_item.childCount()))
        # set icon
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'comm_out.png')))
        # set info
        item.setWTS('Publisher', 'Publisher')
        return item

    def add_subscriber_item(self, data):
        if not self._comm_item:
            self._comm_item = CommunicationsItem(parent=self)
        item = TreeItem(parent=self._comm_item, data=data)
        # update communications count
        self._comm_item.setText(1, str(self._comm_item.childCount()))
        # set icon
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'comm_in.png')))
        # set info
        item.setWTS('Subscriber', 'Subscriber')
        return item

    def add_service_server_item(self, data):
        if not self._comm_item:
            self._comm_item = CommunicationsItem(parent=self)
        item = TreeItem(parent=self._comm_item, key='service server', data=data)
        # update communications count
        self._comm_item.setText(1, str(self._comm_item.childCount()))
        # set icon
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        # set info
        item.setWTS('Service Server', 'Service Server')
        return item

    def add_service_client_item(self, data):
        if not self._comm_item:
            self._comm_item = CommunicationsItem(parent=self)
        item = TreeItem(parent=self._comm_item, key='service client', data=data)
        # update communications count
        self._comm_item.setText(1, str(self._comm_item.childCount()))
        # set icon
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        # set info
        item.setWTS('Service Client', 'Service Client')
        return item

    def add_action_server_item(self, data):
        if not self._comm_item:
            self._comm_item = CommunicationsItem(parent=self)
        item = TreeItem(parent=self._comm_item, key='action server', data=data)
        # update communications count
        self._comm_item.setText(1, str(self._comm_item.childCount()))
        # set icon
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        # set info
        item.setWTS('Action Server', 'Action Server')
        return item

    def add_action_client_item(self, data):
        if not self._comm_item:
            self._comm_item = CommunicationsItem(parent=self)
        item = TreeItem(parent=self._comm_item, key='action client', data=data)
        # update communications count
        self._comm_item.setText(1, str(self._comm_item.childCount()))
        # set icon
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        # set info
        item.setWTS('Action Client', 'Action Client')
        return item

    def add_simple_action_server_item(self, data):
        if not self._comm_item:
            self._comm_item = CommunicationsItem(parent=self)
        item = TreeItem(parent=self._comm_item, key='simple action server', data=data)
        # update communications count
        self._comm_item.setText(1, str(self._comm_item.childCount()))
        # set icon
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        # set info
        item.setWTS('Simple Action Server', 'Simple Action Server')
        return item

    def add_simple_action_client_item(self, data):
        if not self._comm_item:
            self._comm_item = CommunicationsItem(parent=self)
        item = TreeItem(parent=self._comm_item, key='simple action client', data=data)
        # update communications count
        self._comm_item.setText(1, str(self._comm_item.childCount()))
        # set icon
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        # set info
        item.setWTS('Simple Action Client', 'Simple Action Client')
        return item

    def add_tf_broadcaster_item(self, data):
        if not self._tf_item:
            self._tf_item = TransformationsItem(parent=self)
        return TfBroadcasterItem(parent=self._tf_item, data=data)

    def add_tf_listener_item(self, data):
        if not self._tf_item:
            self._tf_item = TransformationsItem(parent=self)
        return TfListenerItem(parent=self._tf_item, data=data)

    def add_state_machine_item(self, data):
        if not self._machines_item:
            self._machines_item = TreeItem(parent=self, type=g.STATE_MACHINES_ITEM)
            self._machines_item.setIcon(0, QIcon(os.path.join(
                rp.get_path('roslab_ide'), 'resource', 'icons', 'state_machine.png')))
        item = StateMachineItem(parent=self._machines_item, data=data, package_name=self._package_name,
                                library_name=self._name, library_item=self)
        self._machines_item.setText(1, str(self._machines_item.childCount()))
        return item

    def add_function_item(self, data):
        if not self._functions_item:
            self._functions_item = FunctionsItem(parent=self)
        function_item = FunctionItem(parent=self._functions_item, data=data)
        self._functions_item.setText(1, str(self._functions_item.childCount()))
        return function_item

    def function_items(self):
        function_items = []
        if self._functions_item:
            count = self._functions_item.childCount()
            for i in range(count):
                function_items.append(self._functions_item.child(i))
        return function_items

    def package_name(self):
        return self._package_name

    @pyqtSlot()
    def preview(self):
        Controller.preview_library(self._package_name, self._name)

    @pyqtSlot()
    def add_import(self):
        data = Controller.add_import(self._package_name, self._name)
        if data:
            self.add_import_item(data=data)

    @pyqtSlot()
    def add_param(self):
        data = Controller.add_parameter(self._package_name, self._name)
        if data:
            self.add_parameter_item(data=data)

    @pyqtSlot()
    def add_basic_comm(self):
        comm, data, callback_data = Controller.add_basic_communication(self._package_name, self._name)
        if comm == 'pub':
            self.add_publisher_item(data=data)
        elif comm == 'sub':
            self.add_subscriber_item(data=data)
        elif comm == 'ss':
            self.add_service_server_item(data=data)
        elif comm == 'sc':
            self.add_service_client_item(data=data)
        else:
            return
        if callback_data:
            self.add_function_item(data=callback_data)

    @pyqtSlot()
    def add_tf_broadcaster(self):
        data = Controller.add_tf_broadcaster(self._package_name, self._name)
        if data:
            self.add_tf_broadcaster_item(data=data)

    @pyqtSlot()
    def add_tf_listener(self):
        data = Controller.add_tf_listener(self._package_name, self._name)
        if data:
            self.add_tf_listener_item(data=data)

    @pyqtSlot()
    def add_state_machine(self):
        data = Controller.add_state_machine(self._package_name, self._name)
        if data:
            self.add_state_machine_item(data=data)

    @pyqtSlot()
    def add_function(self):
        function_data = Controller.add_function(self._package_name, self._name)
        if function_data:
            self.add_function_item(data=function_data)


class NodeItem(TreeItem):

    def __init__(self, parent, data=None, package_name=None):
        TreeItem.__init__(self, parent=parent, type=g.NODE_ITEM, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'executable-script.png')))
        # set info
        self.setWTS('Node', 'Node name')

        # set parent package name
        self._package_name = package_name

        # mod actions
        self.add_action('start', 'konsole1.png', self.start)
        self.add_action('start with args', 'konsole1.png', self.start_with_args)
        # add actions

    @pyqtSlot()
    def start(self):
        Controller.start_node(self._package_name, self._name)

    @pyqtSlot()
    def start_with_args(self):
        Controller.start_node_with_args(self._package_name, self._name)


class RosLaunchFileItem(TreeItem):

    def __init__(self, parent, data=None, package_name=None):
        TreeItem.__init__(self, parent=parent, type=g.ROS_LAUNCH_FILE_ITEM, data=data)

        # set parent package name
        self._package_name = package_name

        # group items
        self._params_item = None
        self._nodes_item = None
        self._includes_item = None

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'ros_launch.png')))
        # set info
        self.setWTS('ROS Launch File', 'ROS Launch File')

        # mod actions
        self.add_action('Start', 'konsole1.png', self.start)
        # add actions
        self.add_action('Parameter', 'param.png', self.add_roslaunch_parameter, add=True)
        self.add_action('Include', 'import.png', self.add_roslaunch_include, add=True)
        self.add_action('Node', 'executable-script.png', self.add_roslaunch_node, add=True)

        if data:
            self._params_item = TreeItem(parent=self)
            self._params_item.setText(0, 'parameters')
            for param in data['params']:
                self.add_roslaunch_parameter_item(data=param)

            self._includes_item = TreeItem(parent=self)
            self._includes_item.setText(0, 'includes')
            for include in data['includes']:
                self.add_roslaunch_include_item(data=include)

            self._nodes_item = TreeItem(parent=self)
            self._nodes_item.setText(0, 'nodes')
            for node in data['nodes']:
                self.add_roslaunch_node_item(data=node)

    def add_roslaunch_parameter_item(self, data):
        item = TreeItem(parent=self._params_item, data=data)
        self._params_item.setText(1, str(self._params_item.childCount()))
        return item

    def add_roslaunch_include_item(self, data):
        item = TreeItem(parent=self._includes_item, data=data)
        self._includes_item.setText(1, str(self._includes_item.childCount()))
        return item

    def add_roslaunch_node_item(self, data):
        item = RosLaunchNodeItem(parent=self._nodes_item, data=data,
                                 package_name=self._package_name, launch_file=self._name)
        self._nodes_item.setText(1, str(self._nodes_item.childCount()))
        return item

    @pyqtSlot()
    def start(self):
        Controller.start_ros_launch(self._package_name, self._name)

    @pyqtSlot()
    def add_roslaunch_parameter(self):
        data = Controller.add_ros_launch_param(self._package_name, self._name)
        if data:
            self.add_roslaunch_parameter_item(data)

    @pyqtSlot()
    def add_roslaunch_include(self):
        data = Controller.add_ros_launch_include(self._package_name, self._name)
        if data:
            self.add_roslaunch_include_item(data)

    @pyqtSlot()
    def add_roslaunch_node(self):
        data = Controller.add_ros_launch_node(self._package_name, self._name)
        if data:
            self.add_roslaunch_node_item(data)


class RosLaunchNodeItem(TreeItem):

    def __init__(self, parent, data, package_name, launch_file):
        TreeItem.__init__(self, parent=parent, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'executable-script.png')))
        # set info
        self.setWTS('ROS Launch Node', 'ROS Launch Node')

        # vars
        self._package_name = package_name
        self._launch_file = launch_file

        # group items
        self._remaps_item = None
        self._params_item = None

        # add actions
        self.add_action('Parameter', 'param.png', self.add_param, add=True)
        self.add_action('Remapping', 'change.png', self.add_remap, add=True)

        if data:
            if 'remap' in data:
                self._remaps_item = TreeItem(parent=self, key='Remappings')
                self._remaps_item.setWTS('Remappings', 'Remappings count')
                for remap in data['remap']:
                    self.add_remap_item(data=remap)
            if 'param' in data:
                self._params_item = TreeItem(parent=self, key='Parameters')
                self._params_item.setWTS('Parameters', 'Parameters count')
                for param in data['param']:
                    self.add_param_item(data=param)

    def add_remap_item(self, data):
        if not self._remaps_item:
            self._remaps_item = TreeItem(parent=self, key='Remappings')
            self._remaps_item.setWTS('Remappings', 'Remappings count')
        item = TreeItem(parent=self._remaps_item, type=g.ROS_LAUNCH_NODE_REMAP_ITEM)
        return item

    def add_param_item(self, data):
        if not self._params_item:
            self._params_item = TreeItem(parent=self, key='Parameters')
            self._params_item.setWTS('Parameters', 'Parameters count')
        item = TreeItem(parent=self._params_item, type=g.ROS_LAUNCH_NODE_PARAM_ITEM)
        return item

    @pyqtSlot()
    def add_remap(self):
        data = Controller.add_ros_launch_node_remap(self._package_name, self._launch_file, self._name)
        if data:
            self.add_remap_item(data)

    @pyqtSlot()
    def add_param(self):
        data = Controller.add_ros_launch_node_param(self._package_name, self._launch_file, self._name)
        if data:
            self.add_param_item(data)


class RoconLaunchFileItem(TreeItem):

    def __init__(self, parent, data=None, package_name=None):
        TreeItem.__init__(self, parent=parent, type=g.ROCON_LAUNCH_FILE_ITEM, data=data)

        # set parent package name
        self._package_name = package_name

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'rocon_launch.png')))
        # set info
        self.setWTS('Robots in Concert Launch File', 'Robots in Concert Launch File')


class TransformationsItem(TreeItem):

    def __init__(self, parent):
        TreeItem.__init__(self, parent=parent)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'tf.png')))
        # set info
        self.setWTS('Transformations', 'Transformations')


class CommunicationsItem(TreeItem):

    def __init__(self, parent):
        TreeItem.__init__(self, parent=parent)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        # set info
        self.setWTS('Communications', 'Communications count')


class TfListenerItem(TreeItem):

    def __init__(self, parent, data):
        TreeItem.__init__(self, parent=parent, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'comm_in.png')))
        # set info
        self.setWTS('Transform Listener', 'Parent -> Child')

        # set item value from data
        self.setText(1, '{0} -> {1}'.format(data['parent_frame'], data['child_frame']))


class TfBroadcasterItem(TreeItem):

    def __init__(self, parent, data):
        TreeItem.__init__(self, parent=parent, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'comm_out.png')))
        # set info
        self.setWTS('Transform Broadcaster', 'Parent -> Child')

        # set item value from data
        self.setText(1, '{0} -> {1}'.format(data['parent_frame'], data['child_frame']))


class FunctionsItem(TreeItem):

    def __init__(self, parent):
        TreeItem.__init__(self, parent=parent)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'functions.png')))
        # set info
        self.setWTS('Functions', 'Functions count')


class FunctionItem(TreeItem):

    def __init__(self, parent, data):
        TreeItem.__init__(self, parent=parent, type=g.FUNCTION_ITEM, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'function.png')))
        # set info
        self.setWTS('Function (class method)', 'Function name')

        # vars
        self._args = ['self']

        # setup data
        if data:
            if 'args' in data:
                for arg in data['args']:
                    self.add_argument_item(data=arg)

    def add_argument_item(self, data):
        self._args.append(data['name'])
        return TreeItem(parent=self, type=g.FUNCTION_ARGUMENT_ITEM, key='argument', data=data)

    def args(self):
        return self._args


class StateMachineItem(TreeItem):

    def __init__(self, parent, data, package_name, library_name, library_item):
        TreeItem.__init__(self, parent=parent, type=g.STATE_MACHINE_ITEM, data=data)

        # vars
        self._package_name = package_name
        self._library_name = library_name
        self._library_item = library_item

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'state_machine.png')))
        # set info
        self.setWTS('Finite State Machine', 'Finite State Machine')

        self._states_item = TreeItem(parent=self, type=g.MACHINE_STATES_ITEM)
        self._states_item.setIcon(0, QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'machine_states.png')))

        # add states
        for state in data['states']:
            self.add_state_item(data=state)

        # mod actions
        visualize_machine_action = QAction('Visualize', None)
        # show_machine_action.setIcon(QIcon(os.path.join(
        #     rp.get_path('roslab_ide'), 'resource', 'icons', 'machine_state.png')))
        visualize_machine_action.triggered.connect(self.visualize_machine)
        self._mod_actions = [visualize_machine_action]
        # add actions
        add_state_action = QAction('State', None)
        add_state_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'machine_state.png')))
        add_state_action.triggered.connect(self.add_state)
        self._add_actions = [add_state_action]

    def package_name(self):
        return self._package_name

    def library_name(self):
        return self._library_name

    def add_state_item(self, data):
        item = StateItem(parent=self._states_item, data=data)
        self._states_item.setText(1, str(self._states_item.childCount()))
        return item

    @pyqtSlot()
    def add_state(self):
        state_data, handler_data = Controller.add_machine_state(self._package_name, self._library_name, self._name)
        if state_data:
            self.add_state_item(data=state_data)
        if handler_data:
            self._library_item.add_function_item(data=handler_data)

    @pyqtSlot()
    def visualize_machine(self):
        Controller.visualize_state_machine(self._package_name, self._library_name, self._name)


class StateItem(TreeItem):

    def __init__(self, parent, data):
        TreeItem.__init__(self, parent=parent, type=g.MACHINE_STATE_ITEM, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'machine_state.png')))
        # set info
        self.setWTS('Machine State', 'Machine State')

        # vars
        self._transitions_item = None
        if 'trans' in data:
            self._transitions_item = TreeItem(parent=self, type=g.STATE_TRANSITIONS_ITEM)
            for transition in data['trans']:
                self.add_transition_item(data=transition)

    def add_transition_item(self, data):
        if not self._transitions_item:
            self._transitions_item = TreeItem(parent=self, type=g.STATE_TRANSITIONS_ITEM)
        return KeyValueItem(parent=self._transitions_item, key='new state', value=data['state'])


class KeyValueItem(TreeItem):

    def __init__(self, parent, data, key, value):
        TreeItem.__init__(self, parent=parent, type=g.KEY_VALUE_ITEM, key=key, value=value)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'key.png')))
        # set info
        self.setWTS('Key', 'Value')

        # mod actions
        edit_action = QAction('Edit', None)
        edit_action.setIcon(QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'tool_properties.png')))
        edit_action.triggered.connect(self.edit_value)
        self._mod_actions = [edit_action]

        # store data
        self._data = data

    @pyqtSlot()
    def edit_value(self):
        key = self.text(0)
        value = self.text(1)
        data = Controller.modify_single_item_data(data=self._data, key=key, current_value=value)
        if data:
            self.setText(1, data[key])


class Controller(object):

    _parent_widget = None

    _workspace_name = None
    _workspace_path = None
    _workspace_item = None
    _workspace_data = {}

    _roslab_item = None
    _rosinstall_item = None

    _has_unsaved_changes = False

    # make it singleton
    _is_initialized = False

    def __init__(self, root_item, parent_widget):
        if Controller._is_initialized:
            raise RuntimeError('Controller is already initialized! Only one instance is allowed!')
        # set main widget
        Controller._parent_widget = parent_widget
        # setup data
        Controller._workspace_data = {'roslab': [], 'rosinstall': [], 'pure': []}
        # workspace root item
        Controller._workspace_item = WorkspaceItem(parent=root_item)
        # setup workspace package list item
        Controller._roslab_item = RoslabPackagesItem(parent=self._workspace_item)
        Controller._rosinstall_item = RosinstallPackagesItem(parent=self._workspace_item)
        # restore settings
        Controller.restore_settings()
        # load workspace
        Controller.load_workspace()
        # make it singleton
        Controller._is_initialized = True

    @staticmethod
    def store_settings():
        settings = QSettings('semCo', 'ROSLab IDE')
        # save global workspace path
        settings.setValue('current_ws_path', Controller._workspace_path)
        print('stored ROS workspace path: {0}'.format(Controller._workspace_path))

    @staticmethod
    def restore_settings():
        settings = QSettings('semCo', 'ROSLab IDE')
        # restore global workspace path
        Controller._workspace_path = str(settings.value('current_ws_path', ''))
        # get first run workspace path
        while Controller._workspace_path is '':
            Controller._workspace_path = str(QFileDialog.getExistingDirectory(caption='Set ROS catkin workspace'))
        print('restored ROS workspace path: {0}'.format(Controller._workspace_path))
        # get workspace name and set it as item text
        Controller._workspace_name = Controller._workspace_path.split('/')[-1]
        Controller._workspace_item.setText(0, Controller._workspace_name)
        Controller._workspace_item.setText(1, Controller._workspace_path)

    @staticmethod
    def workspace_path():
        return Controller._workspace_path

    @staticmethod
    def change_workspace():
        # reset path
        Controller._workspace_path = path = ''
        # get new path
        while Controller._workspace_path is '':
            Controller._workspace_path = str(QFileDialog.getExistingDirectory(caption='Set ROS catkin workspace'))
        print('changed ROS workspace path to: {0}'.format(Controller._workspace_path))
        # store settings
        Controller.store_settings()

    @staticmethod
    def build_workspace():
        ROSCommand.catkin_make(Controller._workspace_path)

    @staticmethod
    def build_package(package):
        ROSCommand.catkin_make(Controller._workspace_path, package)

    @staticmethod
    def data_changed():
        Controller._has_unsaved_changes = True

    @staticmethod
    def has_unsaved_changes():
        return Controller._has_unsaved_changes

    @staticmethod
    def set_workspace_path(workspace_path):
        Controller._workspace_path = workspace_path

    @staticmethod
    def load_rosinstall_packages():
        # clear current listed rosinstall packages from model
        Controller._workspace_data['rosinstall'] = []
        # clear current listed rosinstall packages from view
        for child in Controller._rosinstall_item.takeChildren():
            del child
        # get package folders
        src_dir = os.path.join(Controller._workspace_path, 'src')
        # get .rosinstall packages
        if os.path.exists(os.path.join(src_dir, '.rosinstall')):
            rosinstall_data = g.load_yaml_file(os.path.join(src_dir, '.rosinstall'))
            for package in rosinstall_data:
                package_type = package.keys()[0]
                package_data = {
                    'name': package[package_type]['local-name'],
                    'vcs': package_type,
                    'uri': package[package_type]['uri'],
                    'version': package[package_type]['version']
                }
                # setup workspace data (model)
                Controller._workspace_data['rosinstall'].append(package_data)
                # setup workspace tree (view)
                Controller._rosinstall_item.add_package_item(data=package_data)

    @staticmethod
    def load_roslab_packages():
        # clear current listed rosinstall packages from model
        Controller._workspace_data['roslab'] = []
        # clear current listed rosinstall packages from view
        for child in Controller._roslab_item.takeChildren():
            del child
        # get package folders
        src_dir = os.path.join(Controller._workspace_path, 'src')
        # get roslab packages
        for package_name in os.listdir(src_dir):
            sub_dir = os.path.join(src_dir, package_name)
            if os.path.isdir(sub_dir):
                # look for roslab.yaml in pkg folder
                if 'roslab.yaml' in os.listdir(sub_dir):
                    # get package data
                    package_data = g.load_yaml_file(os.path.join(sub_dir, 'roslab.yaml'))
                    # setup workspace data (model)
                    Controller._workspace_data['roslab'].append(package_data)
                    # setup workspace tree (view)
                    Controller._roslab_item.add_package_item(data=package_data)

    @staticmethod
    def load_workspace():
        # load rosinstall packages
        Controller.load_rosinstall_packages()
        # load roslab packages
        Controller.load_roslab_packages()

    @staticmethod
    def save_workspace():
        # save roslab packages
        for data in Controller._workspace_data['roslab']:
            roslab_yaml = os.path.join(Controller._workspace_path, 'src', data['name'], 'roslab.yaml')
            stream = open(roslab_yaml, 'w')
            yaml.dump(data=data, stream=stream, default_flow_style=False)
            print('saved ROSLab package: {}!'.format(data['name']))
        print('saved workspace: {}!'.format(Controller._workspace_name))

    @staticmethod
    def add_roslab_package():
        # get package name
        package_name, ok = QInputDialog.getText(Controller._parent_widget, 'Add package to workspace', 'name:')
        package_name = str(package_name)
        if not ok or package_name == '':
            return
        # check package directory
        pkg_dir = Controller._workspace_path + '/src/' + package_name
        if os.path.exists(pkg_dir):
            QMessageBox.critical(Controller._parent_widget, 'ROSLab IDE Error!',
                                 'Can not create package, because it already exists!')
            return
        # get backend (mixed packages are not supported yet!)
        backend, ok = QInputDialog.getItem(Controller._parent_widget, 'Add package to workspace',
                                           'language:\n(mixed packages are not supported yet!)', g.backends)
        backend = str(backend)
        if not ok:
            return
        # create dir after all question which can be canceled
        os.mkdir(pkg_dir)
        # add package to managed packages dict
        package_data = {'name': package_name,
                        'roslab_version': __version__,
                        'backend': backend,
                        'libraries': [],
                        'nodes': []
                        }
        if backend == 'python':
            package_data['dependencies'] = [{'name': 'rospy'}]
        elif backend == 'cpp':
            package_data['dependencies'] = [{'name': 'roscpp'}]
        else:
            raise RuntimeError('Unsupported backend! This should not happen!')
        # append workspace data (model)
        Controller._workspace_data['roslab'].append(package_data)
        # append workspace tree (view)
        Controller._roslab_item.add_package_item(data=package_data)
        # create roslab.yaml
        stream = open(pkg_dir + '/roslab.yaml', 'w')
        yaml.dump(package_data, stream=stream, default_flow_style=False)
        Controller.generate_package(package=package_name)
        # give response to user
        print('created new package: {0}!'.format(package_name))

    @staticmethod
    def add_rosinstall_package():
        wstool_set_args = {}
        rosinstall_package_dialog = RosinstallPackageDialog(wstool_set_args, parent=Controller._parent_widget)
        if rosinstall_package_dialog.exec_() == QDialog.Accepted:
            ROSCommand.wstool('set', **wstool_set_args)

    @staticmethod
    def start_node(package, node):
        """
        Start node from package. Rosrun will be called in external process.

        :param package:
        :param node:
        """
        ROSCommand.rosrun(package, node)

    @staticmethod
    def start_node_with_args(package, node):
        """
        Start node from package with arguments. Rosrun will be called in external process.

        :param package:
        :param node:
        """
        # get command line arguments
        args, ok = QInputDialog.getText(Controller._parent_widget, 'Start node with command line args', 'args:')
        args = str(args)
        if not ok or args == '':
            return
        ROSCommand.rosrun(package, node, args)

    @staticmethod
    def start_ros_launch(package, launch_file):
        """
        Start ROS launch file from package.

        :param package:
        :param launch_file:
        """
        ROSCommand.roslaunch(package, launch_file)

    @staticmethod
    def add_dependency(package):
        """
        Add dependency to package.

        :param package:
        :return:
        """
        # get package data
        roslab_packages = Controller._workspace_data['roslab']
        package_data = g.get_dict_list_entry_by_key_value(roslab_packages, 'name', package)
        # get user input
        dependency, ok = QInputDialog.getItem(Controller._parent_widget, 'Add dependency to package',
                                              'dependency name', g.ROS_PKGS)
        dependency = str(dependency)
        if not ok or dependency == '':
            return
        # check existence
        if 'dependencies' not in package_data:
            package_data['dependencies'] = []
        # create data
        dependency_data = {'name': dependency}
        # append list
        package_data['dependencies'].append(dependency_data)
        # mark changed
        Controller.data_changed()
        # return created data
        return dependency_data

    @staticmethod
    def add_library(package):
        # get package data
        """
        Add library to package.

        :param package:
        :return:
        """
        roslab_packages = Controller._workspace_data['roslab']
        package_data = g.get_dict_list_entry_by_key_value(roslab_packages, 'name', package)
        # get user input
        library, ok = QInputDialog.getText(Controller._parent_widget, 'Add library to package', 'library name:')
        library = str(library)
        if not ok or library == '':
            return None
        # check existence
        if 'libraries' not in package_data:
            package_data['libraries'] = []
        # create data
        library_data = {
            'name': library,
            'author': str(g.author),
            'copyright': str(g.place),
            'credits': [],
            'email': str(g.email),
            'info': 'TODO',
            'maintainer': str(g.author),
            'license': 'TODO',
            'status': 'freshly generated',
            'version': '0.0.1'
        }
        # append list
        package_data['libraries'].append(library_data)
        # mark changed
        Controller.data_changed()
        # return created data
        return library_data

    @staticmethod
    def add_node(package):
        """
        Add node to package.

        :param package: Package name where to add node.
        :return: Returns node data if dialogs are accepted, otherwise None
        :rtype: dict or None
        """
        # get package data
        package_data = Controller.get_package_data(package)
        # get libraries from package
        libraries = []
        for library in package_data['libraries']:
            libraries.append(library['name'])
        # return if there are no libraries
        if not len(libraries):
            # TODO: add warning message!
            return None
        # get user input
        node, ok = QInputDialog.getText(Controller._parent_widget, 'Add node to package', 'node name:')
        node = str(node)
        if not ok or node == '':
            return None
        library, ok = QInputDialog.getItem(Controller._parent_widget, 'Set library to call', 'library:', libraries)
        library = str(library)
        if not ok or library == '':
            return None
        # create node data
        node_data = {'name': node, 'library': library}
        # append list
        package_data['nodes'].append(node_data)
        # mark changed
        Controller.data_changed()
        # return created data
        return node_data

    @staticmethod
    def add_ros_launch_file(package):
        # get package data
        package_data = Controller.get_package_data(package)
        file_name, ok = QInputDialog.getText(
            Controller._parent_widget, 'Add ROS launch file to package', 'file name:')
        file_name = str(file_name)
        if not ok or file_name == '':
            return None
        # create roslaunch file data
        ros_launch_data = {
            'name': file_name,
            'params': [],
            'includes': [],
            'nodes': []
        }
        if 'ros_launch' not in package_data:
            package_data['ros_launch'] = []
        # append roslaunch file list
        package_data['ros_launch'].append(ros_launch_data)
        # return created data
        return ros_launch_data

    @staticmethod
    def add_rocon_launch_file(package):
        # get package data
        package_data = Controller.get_package_data(package)
        file_name, ok = QInputDialog.getText(
            Controller._parent_widget, 'Add RoCon launch file to package', 'file name:')
        file_name = str(file_name)
        if not ok or file_name == '':
            return None
        # create roslaunch file data
        rocon_launch_data = {
            'name': file_name
        }
        if 'rocon_launch' not in package_data:
            package_data['rocon_launch'] = []
        # append roslaunch file list
        package_data['rocon_launch'].append(rocon_launch_data)
        # mark changed
        Controller.data_changed()
        # return created data
        return rocon_launch_data

    @staticmethod
    def add_ros_launch_include(package, launch_file):
        # get data
        ros_launch_data = Controller.get_ros_launch_data(package=package, launch_file=launch_file)
        # rospack find is used to find package path
        package_name, ok = QInputDialog.getText(
            Controller._parent_widget, 'Add file include to ROS launch file', 'package:')
        package_name = str(package_name)
        if not ok or package_name == '':
            return None
        file_name, ok = QInputDialog.getText(
            Controller._parent_widget, 'Add file include to ROS launch file', 'file:')
        file_name = str(file_name)
        if not ok or file_name == '':
            return None
        # create data
        include = {
            'package': package_name,
            'file': file_name
        }
        # check if ros laucnh file already has includes list, otherwise create it
        if 'includes' not in ros_launch_data:
            ros_launch_data['includes'] = []
        # append launch file's includes list
        ros_launch_data['includes'].append(include)
        # mark changed
        Controller.data_changed()
        # return data
        return include

    @staticmethod
    def add_ros_launch_param(package, launch_file):
        # get data
        ros_launch_data = Controller.get_ros_launch_data(package=package, launch_file=launch_file)
        # user input
        param_name, ok = QInputDialog.getText(
            Controller._parent_widget, 'Add global parameter to ROS launch file', 'name:')
        param_name = str(param_name)
        if not ok or param_name == '':
            return None
        param_value, ok = QInputDialog.getText(
            Controller._parent_widget, 'Add global parameter to ROS launch file', 'value:')
        param_value = str(param_value)
        if not ok or param_value == '':
            return None
        # create data
        param = {
            'name': param_name,
            'value': param_value
        }
        # check if ros launch file already has parameter list, otherwise create it
        if 'params' not in ros_launch_data:
            ros_launch_data['params'] = []
        # append launch file's includes list
        ros_launch_data['params'].append(param)
        # mark changed
        Controller.data_changed()
        # return data
        return param

    @staticmethod
    def add_ros_launch_node(package, launch_file):
        # get data
        ros_launch_data = Controller.get_ros_launch_data(package=package, launch_file=launch_file)
        node_dialog = RosLaunchNodeDialog(parent=g.main_widget, data={})
        if node_dialog.exec_() == QDialog.Accepted:
            if 'nodes' not in ros_launch_data:
                ros_launch_data['nodes'] = []
            ros_launch_data['nodes'].append(node_dialog.data)
            # mark data changed
            Controller.data_changed()
            return node_dialog.data
        return None

    @staticmethod
    def add_ros_launch_node_remap(package, launch_file, node):
        # get data
        ros_launch_node_data = Controller.get_ros_launch_node_data(package, launch_file, node)
        # user input
        remap_from, ok = QInputDialog.getText(
            Controller._parent_widget, 'Add remapping to ROS launch node', 'from:')
        remap_from = str(remap_from)
        if not ok or remap_from == '':
            return None
        remap_to, ok = QInputDialog.getText(
            Controller._parent_widget, 'Add remapping to ROS launch node', 'to:')
        remap_to = str(remap_to)
        if not ok or remap_to == '':
            return None
        # create data
        remap = {
            'from': remap_from,
            'to': remap_to
        }
        # check if ros launch file already has parameter list, otherwise create it
        if 'remaps' not in ros_launch_node_data:
            ros_launch_node_data['remaps'] = []
        # append launch file's includes list
        ros_launch_node_data['remaps'].append(remap)
        # mark changed
        Controller.data_changed()
        # return data
        return remap

    @staticmethod
    def add_ros_launch_node_param(package, launch_file, node):
        # get data
        ros_launch_node_data = Controller.get_ros_launch_node_data(package, launch_file, node)
        # user input
        param_name, ok = QInputDialog.getText(
            Controller._parent_widget, 'Add private parameter to ROS launch node', 'name:')
        param_name = str(param_name)
        if not ok or param_name == '':
            return None
        param_value, ok = QInputDialog.getText(
            Controller._parent_widget, 'Add private parameter to ROS launch node', 'value:')
        param_value = str(param_value)
        if not ok or param_value == '':
            return None
        # create data
        param = {
            'name': param_name,
            'value': param_value
        }
        # check if ros launch file already has parameter list, otherwise create it
        if 'params' not in ros_launch_node_data:
            ros_launch_node_data['params'] = []
        # append launch file's includes list
        ros_launch_node_data['params'].append(param)
        # mark changed
        Controller.data_changed()
        # return data
        return param

    @staticmethod
    def add_import(package, library):
        """
        Add import to packages' library.

        :type package: str
        :type library: str
        :param package: Package Name
        :param library: Library Name
        :return: Returns import data if dialog is accepted, otherwise None
        :rtype: dict or None
        """
        library_data = Controller.get_library_data(package=package, library=library)
        import_dialog = ImportDialog(parent=g.main_widget)
        if import_dialog.exec_() == QDialog.Accepted:
            # check if import list exists in library, otherwise create it
            if 'import' not in library_data:
                library_data['import'] = []
            library_data['import'].append(import_dialog.data)
            Controller.data_changed()
            # update preview
            Controller.preview_library(package=package, library=library)
            return import_dialog.data
        return None

    @staticmethod
    def add_parameter(package, library):
        """
        Add parameter to packages' library.

        :type package: str
        :type library: str
        :param package: Package Name
        :param library: Library Name
        :return: Returns import data if dialog is accepted, otherwise None
        :rtype: dict or None
        """
        library_data = Controller.get_library_data(package=package, library=library)
        param_dialog = ParameterDialog(parent=g.main_widget)
        if param_dialog.exec_() == QDialog.Accepted:
            # check if parameter list exists in library, otherwise create it
            if 'param' not in library_data:
                library_data['param'] = []
            library_data['param'].append(param_dialog.data)
            # mark workspace data changed
            Controller.data_changed()
            # update preview
            Controller.preview_library(package=package, library=library)
            return param_dialog.data
        return None

    @staticmethod
    def add_basic_communication(package, library):
        """
        Add basic ROS communication to packages' library.

        :type package: str
        :type library: str
        :param package: Package Name
        :param library: Library Name
        :return: Returns comm data, type and callback if dialog is accepted, otherwise None for each
        :rtype: tuple
        """
        library_data = Controller.get_library_data(package=package, library=library)
        comm_dialog = CommunicationDialog(parent=g.main_widget)
        if comm_dialog.exec_() == QDialog.Accepted:
            if 'comm' not in library_data:
                library_data['comm'] = {}
            # check if communication list exists in library, otherwise create it
            if comm_dialog.comm not in library_data['comm']:
                library_data['comm'][comm_dialog.comm] = []
            library_data['comm'][comm_dialog.comm].append(comm_dialog.data)
            Controller.data_changed()
            # check if communication has callback data
            if comm_dialog.callback_data:
                # check if communication list exists in library, otherwise create it
                if 'functions' not in library_data:
                    library_data['functions'] = []
                library_data['functions'].append(comm_dialog.callback_data)
            # update preview
            Controller.preview_library(package=package, library=library)
            return comm_dialog.comm, comm_dialog.data, comm_dialog.callback_data
        return None, None, None

    @staticmethod
    def add_advanced_comm(package, library):
        # TODO: implement me!
        pass

    @staticmethod
    def add_tf_listener(package, library):
        # get library data
        library_data = Controller.get_library_data(package=package, library=library)
        # get user input
        parent, ok = QInputDialog.getText(Controller._parent_widget, 'Add tf listener to package', 'parent frame:')
        parent = str(parent)
        if not ok or parent == '':
            return None
        child, ok = QInputDialog.getText(Controller._parent_widget, 'Add tf listener to package', 'child frame:')
        child = str(child)
        if not ok or child == '':
            return None
        tf_data = {'parent_frame': parent, 'child_frame': child}
        if 'tf' not in library_data:
            library_data['tf'] = {}
        if 'listener' not in library_data['tf']:
            library_data['tf']['listener'] = []
        library_data['tf']['listener'].append(tf_data)
        # update preview
        Controller.preview_library(package=package, library=library)
        return tf_data

    @staticmethod
    def add_tf_broadcaster(package, library):
        # get library data
        library_data = Controller.get_library_data(package=package, library=library)
        # get user input
        parent, ok = QInputDialog.getText(Controller._parent_widget, 'Add tf broadcaster to package', 'parent frame:')
        parent = str(parent)
        if not ok or parent == '':
            return None
        child, ok = QInputDialog.getText(Controller._parent_widget, 'Add tf broadcaster to package', 'child frame:')
        child = str(child)
        if not ok or child == '':
            return None
        tf_data = {'parent_frame': parent, 'child_frame': child}
        if 'tf' not in library_data:
            library_data['tf'] = {}
        if 'broadcaster' not in library_data['tf']:
            library_data['tf']['broadcaster'] = []
        library_data['tf']['broadcaster'].append(tf_data)
        # update preview
        Controller.preview_library(package=package, library=library)
        return tf_data

    @staticmethod
    def add_state_machine(package, library):
        # get library data
        """
        Add Finite State Machine to packages' library.

        :type package: str
        :type library: str
        :param package:
        :param library:
        :return:
        """
        library_data = Controller.get_library_data(package=package, library=library)
        # get machine name
        machine, ok = QInputDialog.getText(Controller._parent_widget, 'Add state machine to library', 'name:')
        machine = str(machine)
        if not ok or machine == '':
            return None
        # setup fsm data
        fsm_data = {
            'name': machine,
            'states': [
                {
                    'name': '{}_start_state'.format(machine),
                    'handler': '{}_start_state_handler'.format(machine)
                },
                {
                    'name': '{}_finished_state'.format(machine)
                }
            ],
            'start_state': '{}_start_state'.format(machine)
        }
        # check if library already has other machine(s)
        if 'fsm' not in library_data:
            library_data['fsm'] = []
        # append machine list
        library_data['fsm'].append(fsm_data)
        # check functions existence
        if 'functions' not in library_data:
            library_data['functions'] = []
        # append functions list with start state handler
        library_data['functions'].append({
            'name': '{}_start_state_handler'.format(machine),
            'args': [{'name': 'cargo', 'default': 'None'}],
            'code': '# TODO: implement me!\nreturn {}_finished_state, None # new_state, new_state_cargo'.format(machine)
        })
        # mark changed
        Controller.data_changed()
        # update preview
        Controller.preview_library(package=package, library=library)
        # return created data
        return fsm_data

    @staticmethod
    def add_machine_state(package, library, machine):
        # TODO: refactor this as dialog with normal/end-state selector
        # get machine data
        machine_data = Controller.get_machine_data(package=package, library=library, machine=machine)
        # get state name
        state, ok = QInputDialog.getText(Controller._parent_widget, 'Add state to machine', 'name:')
        state = str(state)
        if not ok or state == '':
            return None, None
        type_, ok = QInputDialog.getItem(Controller._parent_widget, 'Add state to machine', 'type', ['normal', 'end'])
        if not ok:
            return None, None
        if type_ == 'normal':
            # setup state data
            state_data = {
                'name': '{}_{}_state'.format(machine, state),
                'handler': '{}_{}_state_handler'.format(machine, state)
            }
            handler_data = {
                'name': '{}_{}_state_handler'.format(machine, state),
                'args': [{'name': 'cargo', 'default': 'None'}],
                'code': "# TODO: implement me!\nreturn ('{}_finished_state', None) # next state, cargo".format(machine)
            }
            # get library data
            library_data = Controller.get_library_data(package=package, library=library)
            # check functions existence
            if 'functions' not in library_data:
                library_data['functions'] = []
            # append functions list with state handler
            library_data['functions'].append(handler_data)
        else:
            state_data = {
                'name': '{}_{}_state'.format(machine, state),
            }
            handler_data = None
        # append state list
        machine_data['states'].append(state_data)
        # mark changed
        Controller.data_changed()
        # update preview
        Controller.preview_library(package=package, library=library)
        # update state machine visualization
        Controller.visualize_state_machine(package, library, machine)
        # return created data
        return state_data, handler_data

    @staticmethod
    def add_function(package, library):
        """
        Add function to library in package.

        :type package: str
        :type library: str
        :param package: Package containing library.
        :param library: Library where to add function.
        :return: Returns function data if dialog is accepted, otherwise None.
        :rtype: dict or None
        """
        library_data = Controller.get_library_data(package=package, library=library)
        # get user input
        function, ok = QInputDialog.getText(Controller._parent_widget, 'Add function to library', 'function name:')
        function = str(function)
        if not ok or function == '':
            return None
        # check existence
        if 'functions' not in library_data:
            library_data['functions'] = []
        # create data
        function_data = {'name': function, 'args': [], 'code': '# TODO: implement me!\npass'}
        # append list
        library_data['functions'].append(function_data)
        # mark changed
        Controller.data_changed()
        # update preview
        Controller.preview_library(package=package, library=library)
        # return created data
        return function_data

    @staticmethod
    def remove_roslab_package(package):
        package_list = list(Controller._workspace_data['roslab'])
        for package_data in package_list:
            if package_data['name'] == package:
                package_list.remove(package_data)

    @staticmethod
    def remove_library(package, library):
        # get package data
        """
        Remove library from package.

        :param package:
        :param library:
        """
        package_data = Controller.get_package_data(package)
        for library_data in package_data['libraries']:
            if library_data['name'] == library:
                list(package_data['libraries']).remove(library_data)

    @staticmethod
    def remove_node(package, node):
        # get package data
        """
        Remove node from package.

        :param package:
        :param node:
        """
        package_data = Controller.get_package_data(package)
        for node_data in package_data['nodes']:
            if node_data['name'] == node:
                list(package_data['nodes']).remove(node_data)

    @staticmethod
    def remove_import(package, library, module, include=None):
        # get library data
        """
        Remove module/class-import from packages' library.

        :param package:
        :param library:
        :param module:
        :param include:
        """
        library_data = Controller.get_library_data(package=package, library=library)
        # TODO: implement me!

    @staticmethod
    def remove_parameter(package, library, parameter):
        """
        Remove parameter from packages' library.

        :type package: str
        :type library: str
        :type parameter: str
        :param package:
        :param library:
        :param parameter:
        """
        # get library data
        library_data = Controller.get_library_data(package=package, library=library)
        for param_data in library_data['params']:
            if param_data['name'] == parameter:
                library_data['params'].remove(param_data)

    @staticmethod
    def preview_library(package, library):
        library_data = Controller.get_library_data(package=package, library=library)
        # start = time.time()
        backend = PyBackend(library_data['name'], data=library_data)
        g.preview_widget.parent().parent().setWindowTitle('{} | {}'.format(package, library))
        generated = backend.generate()
        g.preview_widget.setText(generated)
        # save tmp file
        tmp_file = file('/tmp/preview.py', 'w+')
        tmp_file.write(generated)
        tmp_file.flush()
        tmp_file.close()
        # elapsed = (time.time() - start)
        # print('previewing {} from {}... generated in {}'.format(library, package, elapsed))

    @staticmethod
    def visualize_state_machine(package, library, machine):
        # get machine data
        machine_data = Controller.get_machine_data(package=package, library=library, machine=machine)
        start_state = machine_data['start_state']
        graph = pygraphviz.AGraph(directed=True)
        graph.add_node(start_state, color='green', shape='invhouse')
        for state in machine_data['states']:
            state_name = state['name']
            if 'handler' in state:
                if state_name != start_state and graph.has_node(state_name):
                    node = graph.get_node(state_name)
                    node.attr['color'] = 'blue'
                    node.attr['shape'] = 'rect'
                else:
                    graph.add_node(state_name, color='blue', shape='rect')
                handler_data = Controller.get_function_data(package, library, state['handler'])
                handler_code = handler_data['code']
                # parse code return values to get transitions
                return_pattern = re.compile(r'return \(.+\)')
                values_pattern = re.compile(r'\(.+\)')
                handler_returns = return_pattern.findall(handler_code)
                if len(handler_returns):
                    for handler_return in handler_returns:
                        return_tuple = values_pattern.findall(handler_return)[0].replace('(', '').replace(')', '')
                        new_state = return_tuple.split(',')[0].strip().strip("'").strip('"')
                        new_cargo = return_tuple.split(',')[1].strip()
                        graph.add_edge(state_name, new_state)
                else:
                    print('Warning: State has handler, but no transitions. Graph will be incorrect!')
            else:
                if graph.has_node(state_name):
                    node = graph.get_node(state_name)
                    node.attr['color'] = 'red'
                    node.attr['shape'] = 'house'
                else:
                    graph.add_node(state_name, color='red', shape='house')
        graph.layout(prog='dot')
        graph.draw(path='/tmp/fsm.png')
        # fsm_png = graph.draw(format='png')

    @staticmethod
    def get_package_data(package):
        roslab_packages = Controller._workspace_data['roslab']
        package_data = g.get_dict_list_entry_by_key_value(roslab_packages, 'name', package)
        return package_data

    @staticmethod
    def get_ros_launch_data(package, launch_file):
        package_data = Controller.get_package_data(package=package)
        roslaunch_files = package_data['ros_launch']
        roslaunch_data = g.get_dict_list_entry_by_key_value(roslaunch_files, 'name', launch_file)
        return roslaunch_data

    @staticmethod
    def get_ros_launch_node_data(package, launch_file, node):
        ros_launch_data = Controller.get_ros_launch_data(package, launch_file)
        nodes_files = ros_launch_data['nodes']
        node_data = g.get_dict_list_entry_by_key_value(nodes_files, 'name', node)
        return node_data

    @staticmethod
    def get_rocon_launch_data(package, launch_file):
        package_data = Controller.get_package_data(package=package)
        roconlaunch_files = package_data['rocon_launch']
        roslaunch_data = g.get_dict_list_entry_by_key_value(roconlaunch_files, 'name', launch_file)
        return roslaunch_data

    @staticmethod
    def get_library_data(package, library):
        package_data = Controller.get_package_data(package=package)
        roslab_libraries = package_data['libraries']
        library_data = g.get_dict_list_entry_by_key_value(roslab_libraries, 'name', library)
        return library_data

    @staticmethod
    def get_machine_data(package, library, machine):
        library_data = Controller.get_library_data(package=package, library=library)
        library_fsm = library_data['fsm']
        machine_data = g.get_dict_list_entry_by_key_value(library_fsm, 'name', machine)
        return machine_data

    @staticmethod
    def get_function_data(package, library, function):
        library_data = Controller.get_library_data(package=package, library=library)
        library_functions = library_data['functions']
        function_data = g.get_dict_list_entry_by_key_value(library_functions, 'name', function)
        return function_data

    @staticmethod
    def modify_single_item_data(data, key, current_value):
        # get new value for key
        new_value, ok = QInputDialog.getText(Controller._parent_widget, 'Edit ' + key, 'New value:',
                                             QLineEdit.Normal, current_value)
        # validate
        new_value = str(new_value)
        if not ok or new_value == '':
            return None
        # set new value in dict
        data[key] = new_value
        # mark data as changed
        Controller.data_changed()
        # return modified data
        return data

    @staticmethod
    def generate_package(package):
        package_data = Controller.get_package_data(package)
        package_path = os.path.join(Controller._workspace_path, 'src', package)
        print 'Generating {} in {}'.format(package, package_path)
        # update package.xml
        print('Generating package.xml...')
        package_xml = file(os.path.join(package_path, 'package.xml'), 'w+')
        package_xml.write(PackageXmlGenerator(package, build_depends=package_data['dependencies'],
                          run_depends=package_data['dependencies']).generate())
        package_xml.flush()
        package_xml.close()
        print('...done')
        # update CMakeLists.txt
        python_setup = False
        if len(package_data['libraries']):
            python_setup = True
        print('Generating CMakeLists.txt...')
        cmakelists_txt = file(os.path.join(package_path, 'CMakeLists.txt'), 'w+')
        cmakelists_txt.write(CMakeListsTxtGenerator(package, catkin_depends=package_data['dependencies'],
                             python_setup=python_setup, python_scripts=package_data['nodes']).generate())
        cmakelists_txt.flush()
        cmakelists_txt.close()
        print('...done')
        # update setup.py if needed
        if len(package_data['libraries']):
            print('Generating setup.py...')
            setup_py = file(os.path.join(package_path, 'setup.py'), 'w+')
            setup_py.write(SetupPyGenerator(package, requirements=package_data['dependencies']).generate())
            setup_py.flush()
            setup_py.close()
            print('...done')
            # generate libraries
            if not os.path.exists(os.path.join(package_path, 'src')):
                os.mkdir(os.path.join(package_path, 'src'))
            if not os.path.exists(os.path.join(package_path, 'src', package)):
                os.mkdir(os.path.join(package_path, 'src', package))
            open(os.path.join(package_path, 'src', package, '__init__.py'), 'w+')
            for library_data in package_data['libraries']:
                print('Generating library {}...'.format(library_data['name']))
                library_file = file(os.path.join(package_path, 'src', package, '{}.py'.format(library_data['name'])),
                                    'w+')
                library_file.write(PyBackend(library_data['name'], data=library_data).generate())
                library_file.flush()
                library_file.close()
                print('...done')
        # generate nodes
        if len(package_data['nodes']):
            if not os.path.exists(os.path.join(package_path, 'scripts')):
                os.mkdir(os.path.join(package_path, 'scripts'))
            for node_data in package_data['nodes']:
                print('Generating {}_node for library {}...'.format(node_data['name'], node_data['library']))
                node_path = os.path.join(package_path, 'scripts', '{}_node'.format(node_data['name']))
                node_file = file(node_path, 'w+')
                node_file.write(PyBackend.generate_node(node_data['name'], package, node_data['library']))
                node_file.flush()
                node_file.close()
                print('...done')
                print('Mark {}_node as executable...'.format(node_data['name']))
                node_stat = os.stat(node_path)
                os.chmod(node_path, node_stat.st_mode | stat.S_IEXEC)
                print('...done')
        # generate ROS launch files
        if len(package_data['ros_launch']):
            if not os.path.exists(os.path.join(package_path, 'launch')):
                os.mkdir(os.path.join(package_path, 'launch'))
            for ros_launch_data in package_data['ros_launch']:
                print('Generating {}.launch ...'.format(ros_launch_data['name']))
                ros_launch_path = os.path.join(package_path, 'launch', '{}.launch'.format(ros_launch_data['name']))
                ros_launch_file = file(ros_launch_path, 'w+')
                ros_launch_file.write(RosLaunchFileGenerator(ros_launch_data).generate())
                ros_launch_file.flush()
                ros_launch_file.close()
                print('...done')

        print('Successfully generated package: {}'.format(package))