__author__ = 'privat'

import os
import stat
import yaml

import rospkg
rp = rospkg.RosPack()

# pyqt imports
from PyQt4.QtCore import pyqtSlot, QSettings
from PyQt4.QtGui import QAction, QMenu, QTreeWidgetItem, QIcon, QLineEdit
from PyQt4.QtGui import QDialog, QInputDialog, QFileDialog, QTreeWidgetItem, QMessageBox

import roslab_ide.helper.globals as g

from roslab_ide.dialogs.RosinstallPackageDialog import RosinstallPackageDialog
from roslab_ide.dialogs.ArgumentDialog import ArgumentDialog
from roslab_ide.dialogs.ImportDialog import ImportDialog
from roslab_ide.dialogs.ParameterDialog import ParameterDialog
from roslab_ide.dialogs.CommunicationDialog import CommunicationDialog

from roslab_ide.helper.ROSCommand import ROSCommand
from roslab_ide.backends.PyBackend import PyBackend
from roslab_ide.generators.CMakeListsTxtGenerator import CMakeListsTxtGenerator
from roslab_ide.generators.PackageXmlGenerator import PackageXmlGenerator
from roslab_ide.generators.SetupPyGenerator import SetupPyGenerator
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
        self.setToolTip(0, 'Current workspace')
        self.setToolTip(1, 'Workspace path')
        self.setStatusTip(0, 'Current workspace')
        self.setStatusTip(1, 'Workspace path')
        self.setWhatsThis(0, 'Current workspace')
        self.setWhatsThis(1, 'Workspace path')

        # setup mod actions
        change_workspace_action = QAction('change', None)
        change_workspace_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'change.png')))
        change_workspace_action.triggered.connect(self.change_workspace)
        build_workspace_action = QAction('build', None)
        build_workspace_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'build.png')))
        build_workspace_action.triggered.connect(self.build_workspace)
        self._mod_actions = [
            change_workspace_action,
            build_workspace_action
        ]
        # setup add actions
        add_roslab_package_action = QAction('ROSLab package', None)
        add_roslab_package_action.triggered.connect(self.add_roslab_package)
        add_roslab_package_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'open-box.png')))
        add_rosinstall_package_action = QAction('rosinstall package', None)
        add_rosinstall_package_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'box.png')))
        add_rosinstall_package_action.triggered.connect(self.add_rosinstall_package)
        self._add_actions = [
            add_roslab_package_action,
            add_rosinstall_package_action
        ]

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
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'package.png')))
        # set info
        self.setToolTip(0, 'ROSLab packages')
        self.setToolTip(1, 'ROSLab package count')
        self.setStatusTip(0, 'ROSLab packages')
        self.setStatusTip(1, 'ROSLab package count')
        self.setWhatsThis(0, 'ROSLab packages')
        self.setWhatsThis(1, 'ROSLab package count')

        self._workspace_item = parent

        # mod actions
        # add actions
        add_package_action = QAction('Package', None)
        add_package_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'open-box.png')))
        add_package_action.triggered.connect(self._workspace_item.add_roslab_package)
        self._add_actions = [
            add_package_action
        ]

    def add_package_item(self, data):
        return RoslabPackageItem(parent=self, data=data)


class RoslabPackageItem(TreeItem):

    def __init__(self, parent, data=None):
        TreeItem.__init__(self, parent=parent, type=g.ROSLAB_PACKAGE_ITEM, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'open-box.png')))
        # set info
        self.setToolTip(0, 'ROSLab package')
        self.setToolTip(1, 'ROSLab package name')
        self.setStatusTip(0, 'ROSLab package')
        self.setStatusTip(1, 'ROSLab package name')
        self.setWhatsThis(0, 'ROSLab package')
        self.setWhatsThis(1, 'ROSLab package name')

        # mod actions
        generate_package_action = QAction('generate', None)
        generate_package_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'build.png')))
        generate_package_action.triggered.connect(self.generate)
        self._mod_actions = [
            generate_package_action
        ]
        # add actions
        add_dependency_action = QAction('Dependency', None)
        add_dependency_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'dependency.png')))
        add_dependency_action.triggered.connect(self.add_dependency)
        add_library_action = QAction('Library', None)
        add_library_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'binary.png')))
        add_library_action.triggered.connect(self.add_library)
        add_node_action = QAction('Node', None)
        add_node_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'executable-script.png')))
        add_node_action.triggered.connect(self.add_node)
        self._add_actions = [
            add_dependency_action,
            add_library_action,
            add_node_action
        ]

        if data:
            # set package name as item value
            self._name = data['name']
            self.setText(1, self._name)
            # add child items
            if 'dependencies' in data:
                for dependency in data['dependencies']:
                    self.add_dependency_item(data=dependency)
            if 'libraries' in data:
                for library in data['libraries']:
                    self.add_library_item(data=library)
            if 'nodes' in data:
                for node in data['nodes']:
                    self.add_node_item(data=node)

    def add_dependency_item(self, data):
        return DependencyItem(parent=self, data=data, package_name=self._name)

    def add_library_item(self, data):
        return LibraryItem(parent=self, data=data, package_name=self._name)

    def add_node_item(self, data):
        return NodeItem(parent=self, data=data, package_name=self._name)

    @pyqtSlot()
    def generate(self):
        Controller.generate_package(package=self._name)

    @pyqtSlot()
    def add_dependency(self):
        data = Controller.add_dependency(self._name)
        self.add_dependency_item(data=data)

    @pyqtSlot()
    def add_library(self):
        data = Controller.add_library(self._name)
        self.add_library_item(data=data)

    @pyqtSlot()
    def add_node(self):
        data = Controller.add_node(self._name)
        self.add_node_item(data=data)


class RosinstallPackagesItem(TreeItem):

    def __init__(self, parent):
        TreeItem.__init__(self, parent=parent, type=g.ROSINSTALL_ITEM, key='rosinstall')

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'package.png')))
        # set info
        self.setToolTip(0, 'rosinstall packages')
        self.setToolTip(1, 'rosinstall package count')
        self.setStatusTip(0, 'rosinstall packages')
        self.setStatusTip(1, 'rosinstall package count')
        self.setWhatsThis(0, 'rosinstall packages')
        self.setWhatsThis(1, 'rosinstall package count')

        self._workspace_item = parent

        # mod actions
        # add actions
        add_package_action = QAction('Package', None)
        add_package_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'box.png')))
        add_package_action.triggered.connect(self._workspace_item.add_rosinstall_package)
        self._add_actions = [add_package_action]

    def add_package_item(self, data):
        return RosinstallPackageItem(parent=self, data=data)


class RosinstallPackageItem(TreeItem):

    def __init__(self, parent, data=None):
        TreeItem.__init__(self, parent=parent, type=g.ROSINSTALL_PACKAGE_ITEM, data=data)
        # set name
        self._name = 'rosinstall'
        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'box.png')))
        # set info
        self.setToolTip(0, 'rosinstall package')
        self.setToolTip(1, 'rosinstall package name')
        self.setStatusTip(0, 'rosinstall package')
        self.setStatusTip(1, 'rosinstall package name')
        self.setWhatsThis(0, 'rosinstall package')
        self.setWhatsThis(1, 'rosinstall package name')


class DependencyItem(TreeItem):

    def __init__(self, parent, data=None, package_name=None):
        TreeItem.__init__(self, parent=parent, type=g.DEPENDENCY_ITEM, data=data)

        # set parent package name
        self._package_name = package_name

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'dependency.png')))
        # set info
        self.setToolTip(0, 'Dependency')
        self.setToolTip(1, 'Dependency name')
        self.setStatusTip(0, 'Dependency')
        self.setStatusTip(1, 'Dependency name')
        self.setWhatsThis(0, 'Dependency')
        self.setWhatsThis(1, 'Dependency name')


class LibraryItem(TreeItem):

    def __init__(self, parent, data, package_name):
        TreeItem.__init__(self, parent=parent, type=g.LIBRARY_ITEM, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'binary.png')))
        # set info
        self.setToolTip(0, 'Library')
        self.setToolTip(1, 'Library name')
        self.setStatusTip(0, 'Library')
        self.setStatusTip(1, 'Library name')
        self.setWhatsThis(0, 'Library')
        self.setWhatsThis(1, 'Library name')

        # set parent package name
        self._package_name = package_name
        self._function_items = []
        self._tf_item = None
        self._tf_items = []

        # mod actions
        # add actions
        add_import_action = QAction('Import', None)
        add_import_action.setIcon(QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'import.png')))
        add_import_action.triggered.connect(self.add_import)
        add_param_action = QAction('Parameter', None)
        add_param_action.setIcon(QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'param.png')))
        add_param_action.triggered.connect(self.add_param)
        add_basic_comm_action = QAction('Basic Communication', None)
        add_basic_comm_action.setIcon(QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons',
                                                         'communication.png')))
        add_basic_comm_action.triggered.connect(self.add_basic_comm)
        add_tf_broadcaster_action = QAction('Tf Broadcaster', None)
        add_tf_broadcaster_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'comm_out.png')))
        add_tf_broadcaster_action.triggered.connect(self.add_tf_broadcaster)
        add_tf_listener_action = QAction('Tf Listener', None)
        add_tf_listener_action.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'comm_in.png')))
        add_tf_listener_action.triggered.connect(self.add_tf_listener)
        add_function_action = QAction('Function', None)
        add_function_action.setIcon(QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'function.png')))
        add_function_action.triggered.connect(self.add_function)
        self._add_actions = [
            add_import_action,
            add_param_action,
            add_basic_comm_action,
            add_tf_broadcaster_action,
            add_tf_listener_action,
            add_function_action
        ]

        # setup data
        if data:
            # add items
            if 'import' in data:
                for entry in data['import']:
                    self.add_import_item(data=entry)
            if 'param' in data:
                for entry in data['param']:
                    self.add_parameter_item(data=entry)
            if 'pub' in data:
                for entry in data['pub']:
                    self.add_publisher_item(data=entry)
            if 'sub' in data:
                for entry in data['sub']:
                    self.add_subscriber_item(data=entry)
            if 'ss' in data:
                for entry in data['ss']:
                    self.add_service_server_item(data=entry)
            if 'sc' in data:
                for entry in data['sc']:
                    self.add_service_client_item(data=entry)
            if 'as' in data:
                for entry in data['as']:
                    self.add_action_server_item(data=entry)
            if 'ac' in data:
                for entry in data['ac']:
                    self.add_action_client_item(data=entry)
            if 'sas' in data:
                for entry in data['sas']:
                    self.add_simple_action_server_item(data=entry)
            if 'sac' in data:
                for entry in data['sac']:
                    self.add_simple_action_client_item(data=entry)
            if 'tf' in data:
                self._tf_item = TreeItem(parent=self)
                self._tf_item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'tf.png')))
                if 'broadcaster' in data['tf']:
                    for entry in data['tf']['broadcaster']:
                        self.add_tf_broadcaster_item(data=entry)
                if 'listener' in data['tf']:
                    for entry in data['tf']['listener']:
                        self.add_tf_listener_item(data=entry)
            if 'functions' in data:
                for entry in data['functions']:
                    self.add_function_item(data=entry)

    def add_import_item(self, data):
        param_item = TreeItem(parent=self, data=data)
        param_item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'import.png')))
        return param_item

    def add_parameter_item(self, data):
        param_item = TreeItem(parent=self, data=data)
        param_item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'param.png')))
        return param_item

    def add_publisher_item(self, data):
        item = TreeItem(parent=self, data=data)
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'comm_out.png')))
        return item

    def add_subscriber_item(self, data):
        item = TreeItem(parent=self, data=data)
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'comm_in.png')))
        return item

    def add_service_server_item(self, data):
        item = TreeItem(parent=self, key='service server', data=data)
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        return item

    def add_service_client_item(self, data):
        item = TreeItem(parent=self, key='service client', data=data)
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        return item

    def add_action_server_item(self, data):
        item = TreeItem(parent=self, key='action server', data=data)
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        return item

    def add_action_client_item(self, data):
        item = TreeItem(parent=self, key='action client', data=data)
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        return item

    def add_simple_action_server_item(self, data):
        item = TreeItem(parent=self, key='simple action server', data=data)
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        return item

    def add_simple_action_client_item(self, data):
        item = TreeItem(parent=self, key='simple action client', data=data)
        item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'communication.png')))
        return item

    def add_tf_broadcaster_item(self, data):
        if not self._tf_item:
            self._tf_item = TreeItem(parent=self)
            self._tf_item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'tf.png')))
        return TfBroadcasterItem(parent=self._tf_item, data=data)

    def add_tf_listener_item(self, data):
        if not self._tf_item:
            self._tf_item = TreeItem(parent=self)
            self._tf_item.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'tf.png')))
        return TfListenerItem(parent=self._tf_item, data=data)

    def add_function_item(self, data):
        function_item = FunctionItem(parent=self, data=data)
        self._function_items.append(function_item)
        return function_item

    def function_items(self):
        return self._function_items

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
        self.setToolTip(0, 'Node')
        self.setToolTip(1, 'Node name')
        self.setStatusTip(0, 'Node')
        self.setStatusTip(1, 'Node name')
        self.setWhatsThis(0, 'Node')
        self.setWhatsThis(1, 'Node name')

        # set parent package name
        self._package_name = package_name

        # mod actions
        start_action = QAction('start', None)
        start_action.setIcon(QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'konsole1.png')))
        start_action.triggered.connect(self.start)
        self._mod_actions = [start_action]
        # add actions

    @pyqtSlot()
    def start(self):
        Controller.start_node(self._package_name, self._name)


class TfListenerItem(TreeItem):

    def __init__(self, parent, data):
        TreeItem.__init__(self, parent=parent, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'comm_in.png')))
        # set info
        self.setToolTip(0, 'Transform Listener')
        self.setToolTip(1, 'Parent -> Child')
        self.setStatusTip(0, 'Transform Listener')
        self.setStatusTip(1, 'Parent -> Child')
        self.setWhatsThis(0, 'Transform Listener')
        self.setWhatsThis(1, 'Parent -> Child')

        # set item value from data
        self.setText(1, '{0} -> {1}'.format(data['parent_frame'], data['child_frame']))


class TfBroadcasterItem(TreeItem):

    def __init__(self, parent, data):
        TreeItem.__init__(self, parent=parent, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'comm_out.png')))
        # set info
        self.setToolTip(0, 'Transform Broadcaster')
        self.setToolTip(1, 'Parent -> Child')
        self.setStatusTip(0, 'Transform Broadcaster')
        self.setStatusTip(1, 'Parent -> Child')
        self.setWhatsThis(0, 'Transform Broadcaster')
        self.setWhatsThis(1, 'Parent -> Child')

        # set item value from data
        self.setText(1, '{0} -> {1}'.format(data['parent_frame'], data['child_frame']))


class FunctionItem(TreeItem):

    def __init__(self, parent, data):
        TreeItem.__init__(self, parent=parent, type=g.FUNCTION_ITEM, data=data)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'function.png')))
        # set info
        self.setToolTip(0, 'Function (class method)')
        self.setToolTip(1, 'Function name')
        self.setStatusTip(0, 'Function (class method)')
        self.setStatusTip(1, 'Function name')
        self.setWhatsThis(0, 'Function (class method)')
        self.setWhatsThis(1, 'Function name')

        # setup data
        if data:
            if 'args' in data:
                for arg in data['args']:
                    self.add_argument_item(data=arg)

    def add_argument_item(self, data):
        return TreeItem(parent=self, type=g.FUNCTION_ARGUMENT_ITEM, key='argument', data=data)


class KeyValueItem(TreeItem):

    def __init__(self, parent, data, key, value):
        TreeItem.__init__(self, parent=parent, type=g.KEY_VALUE_ITEM, key=key, value=value)

        # set icon
        self.setIcon(0, QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'key.png')))
        # set info
        self.setToolTip(0, 'Key')
        self.setToolTip(1, 'Value')
        self.setStatusTip(0, 'Key')
        self.setStatusTip(1, 'Value')
        self.setWhatsThis(0, 'Key')
        self.setWhatsThis(1, 'Value')

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
                    'cvs': package_type,
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
    def remove_roslab_package(package):
        package_list = list(Controller._workspace_data['roslab'])
        for package_data in package_list:
            if package_data['name'] == package:
                package_list.remove(package_data)

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
    def add_dependency(package):
        # get package data
        """
        Add dependency to package.

        :param package:
        :return:
        """
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
            return import_dialog.data
        return None

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
            Controller.data_changed()
            return param_dialog.data
        return None

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
            # check if communication list exists in library, otherwise create it
            if comm_dialog.comm not in library_data:
                library_data[comm_dialog.comm] = []
            library_data[comm_dialog.comm].append(comm_dialog.data)
            Controller.data_changed()
            # check if communication has callback data
            if comm_dialog.callback_data:
                # check if communication list exists in library, otherwise create it
                if 'functions' not in library_data:
                    library_data['functions'] = []
                library_data['functions'].append(comm_dialog.callback_data)
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
        return tf_data

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
        # return created data
        return function_data

    @staticmethod
    def preview_library(package, library):
        library_data = Controller.get_library_data(package=package, library=library)
        backend = PyBackend(library_data['name'], data=library_data)
        g.preview_widget.setText(backend.generate())

    @staticmethod
    def get_package_data(package):
        roslab_packages = Controller._workspace_data['roslab']
        package_data = g.get_dict_list_entry_by_key_value(roslab_packages, 'name', package)
        return package_data

    @staticmethod
    def get_library_data(package, library):
        package_data = Controller.get_package_data(package=package)
        roslab_libraries = package_data['libraries']
        library_data = g.get_dict_list_entry_by_key_value(roslab_libraries, 'name', library)
        return library_data

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
        # update package.xml
        package_xml = file(os.path.join(package_path, 'package.xml'), 'w+')
        package_xml.write(PackageXmlGenerator(package, build_depends=package_data['dependencies'],
                          run_depends=package_data['dependencies']).generate())
        package_xml.flush()
        package_xml.close()
        # update CMakeLists.txt
        python_setup = False
        if len(package_data['libraries']):
            python_setup = True
        cmakelists_txt = file(os.path.join(package_path, 'CMakeLists.txt'), 'w+')
        cmakelists_txt.write(CMakeListsTxtGenerator(package, catkin_depends=package_data['dependencies'],
                             python_setup=python_setup, python_scripts=package_data['nodes']).generate())
        cmakelists_txt.flush()
        cmakelists_txt.close()
        # update setup.py if needed
        if len(package_data['libraries']):
            setup_py = file(os.path.join(package_path, 'setup.py'), 'w+')
            setup_py.write(SetupPyGenerator(package, requirements=package_data['dependencies']).generate())
            setup_py.flush()
            setup_py.close()
            # generate libraries
            if not os.path.exists(os.path.join(package_path, 'src')):
                os.mkdir(os.path.join(package_path, 'src'))
            if not os.path.exists(os.path.join(package_path, 'src', package)):
                os.mkdir(os.path.join(package_path, 'src', package))
            open(os.path.join(package_path, 'src', package, '__init__.py'), 'w+')
            for library_data in package_data['libraries']:
                library_file = file(os.path.join(package_path, 'src', package, '{}.py'.format(library_data['name'])),
                                    'w+')
                library_file.write(PyBackend(library_data['name'], data=library_data).generate())
                library_file.flush()
                library_file.close()
        # generate nodes
        if len(package_data['nodes']):
            if not os.path.exists(os.path.join(package_path, 'scripts')):
                os.mkdir(os.path.join(package_path, 'scripts'))
            for node_data in package_data['nodes']:
                node_path = os.path.join(package_path, 'scripts', '{}_node'.format(node_data['name']))
                node_file = file(node_path, 'w+')
                node_file.write(PyBackend.generate_node(node_data['name'], package, node_data))
                node_file.flush()
                node_file.close()
                node_stat = os.stat(node_path)
                os.chmod(node_path, node_stat.st_mode | stat.S_IEXEC)