__author__ = 'privat'

import os
import yaml
import stat
import rosmsg
import rospkg
rp = rospkg.RosPack()
from catkin.find_in_workspaces import find_in_workspaces

# pyqt imports
from PyQt4.QtGui import QProgressDialog, QTreeWidgetItem, QMessageBox, QColor
from PyQt4.QtCore import Qt, QSettings

# settings
name = 'Change name in global settings'
author = 'Change name in author settings'
email = 'Change name in email settings'
place = 'Change name in place settings'
license_ = 'Change name in license settings'

keep_functions_open = False

# generator backends
backends = ['python']

# general
main_widget = None
preview_widget = None
cursor_color = QColor(127, 127, 127)

# messages & services
ROS_PKGS = []
ROS_MSGS = []
ROS_SRVS = []


# make nice literal text while yaml dump
class literal_str(str):
    pass


def get_ros_env():
    ros_env = {}
    for k, v in os.environ.iteritems():
        if 'ROS' in k:
            ros_env[k] = v
    return ros_env


def get_dict_list_entry_by_key_value(list_, key, value):
    if type(list_) is not list:
        raise TypeError('Given list is not from type list!')
    for entry in list_:
        if entry[key] == value:
            return entry
    raise TypeError('Entry not found! Check if you are looking in correct list!\n' +
                    'Key/Value: {}/{}\n'.format(key, value) +
                    'Entries: {}'.format(list_))


def get_child_item_by_key(parent, key):
    for index in range(parent.childCount()):
        child = parent.child(index)
        if child.text(0) == key:
            return child
    return None


def check_topic(topic):
    if type(topic) is not str:
        topic = str(topic)
    return '/' + topic if topic[0] is not '/' else topic


def clean_topic(topic):
    topic = check_topic(topic)
    return topic.replace('/', '_')


def load_yaml_file(roslab_yaml):
    # get data from yaml file
    stream = open(roslab_yaml, 'r')
    return yaml.load(stream=stream)


def get_package_executables(package):
    executables = []
    package_directories = find_in_workspaces(project=package)
    for package_directory in package_directories:
        for root, dirs, file_names in os.walk(package_directory):
            # exclude invisible directories
            if '/.' in root:
                continue
            for file_name in file_names:
                if os.stat(os.path.join(root, file_name)).st_mode & stat.S_IEXEC:
                    executables.append(file_name)
    return executables


def scan_ros_distro():
    global main_widget
    global ROS_PKGS, ROS_MSGS, ROS_SRVS
    # reset current
    ROS_MSGS = []
    ROS_SRVS = []
    # scan with progress shown
    progress_dialog = QProgressDialog('Scanning ROS Distro', 'Cancel', 0, len(ROS_PKGS), parent=main_widget)
    progress_dialog.setWindowModality(Qt.WindowModal)
    count = 0
    total = len(ROS_PKGS)
    for package in ROS_PKGS:
        if progress_dialog.wasCanceled():
            ROS_MSGS = []
            ROS_SRVS = []
            break
        progress_dialog.setValue(count)
        progress_dialog.setLabelText('Scanning ROS Distro\nscanning {0} from {1}: {2}...'.format(
            count+1, total, package))
        if len(rosmsg.list_msgs(package)) > 0:
            ROS_MSGS.append(package)
            progress_dialog.setLabelText('Scanning ROS Distro\nscanning {0} from {1}: {2}...\nfound messages...'.format(
                count+1, total, package))
        if len(rosmsg.list_srvs(package)) > 0:
            progress_dialog.setLabelText('Scanning ROS Distro\nscanning {0} from {1}: {2}...\nfound services...'.format(
                count+1, total, package))
            ROS_SRVS.append(package)
        count += 1
    progress_dialog.setValue(len(ROS_PKGS))
    # store settings
    store_settings()
    print 'stored {0} msgs and {1} srvs...'.format(len(ROS_MSGS), len(ROS_SRVS))


def restore_settings():
    global main_widget
    global name, email, place
    global ROS_PKGS, ROS_MSGS, ROS_SRVS
    settings = QSettings('semCo', 'ROSLab IDE')
    name = settings.value('name', 'Frodo')
    email = settings.value('email', 'frodo@middle.earth')
    place = settings.value('place', 'Middle Earth')
    # get message packages
    size = settings.beginReadArray('msg_packages')
    for i in range(size):
        settings.setArrayIndex(i)
        name = settings.value('name', '')
        ROS_MSGS.append(name)
    settings.endArray()
    if len(ROS_MSGS):
        print('found {0} packages containing messages'.format(len(ROS_MSGS)))
    # get service packages
    size = settings.beginReadArray('srv_packages')
    for i in range(size):
        settings.setArrayIndex(i)
        name = settings.value('name', '')
        ROS_SRVS.append(name)
    settings.endArray()
    if len(ROS_SRVS):
        print('found {0} packages containing services'.format(len(ROS_SRVS)))
    if not (len(ROS_MSGS) or len(ROS_SRVS)):
        QMessageBox.information(main_widget, 'ROSLab IDE', 'Could not load message & service packages. Will scan now!')
        scan_ros_distro()


def store_settings():
    global name, email, place
    global ROS_PKGS, ROS_MSGS, ROS_SRVS
    settings = QSettings('semCo', 'ROSLab IDE')
    # store user information
    settings.setValue('name', name)
    settings.setValue('email', email)
    settings.setValue('place', place)
    # store message packages
    size = len(ROS_MSGS)
    settings.beginWriteArray('msg_packages', size)
    for i in range(size):
        settings.setArrayIndex(i)
        settings.setValue('name', ROS_MSGS[i])
    settings.endArray()
    # store service packages
    size = len(ROS_SRVS)
    settings.beginWriteArray('srv_packages', size)
    for i in range(size):
        settings.setArrayIndex(i)
        settings.setValue('name', ROS_SRVS[i])
    settings.endArray()

# undefined item
UNDEFINED_ITEM = QTreeWidgetItem.UserType

# workspace (top level)
WORKSPACE_ITEM = QTreeWidgetItem.UserType + 1

# package roots (workspace children)
ROSLAB_ITEM = QTreeWidgetItem.UserType + 2
ROSINSTALL_ITEM = QTreeWidgetItem.UserType + 3
UNMANAGED_ITEM = QTreeWidgetItem.UserType + 4

# package items (package root children)
ROSLAB_PACKAGE_ITEM = QTreeWidgetItem.UserType + 11
ROSINSTALL_PACKAGE_ITEM = QTreeWidgetItem.UserType + 12
UNMANAGED_PACKAGE_ITEM = QTreeWidgetItem.UserType + 13

# roslab package child items
DEPENDENCIES_ITEM = QTreeWidgetItem.UserType + 21
DEPENDENCY_ITEM = QTreeWidgetItem.UserType + 22

LIBRARIES_ITEM = QTreeWidgetItem.UserType + 23
LIBRARY_ITEM = QTreeWidgetItem.UserType + 24

NODES_ITEM = QTreeWidgetItem.UserType + 25
NODE_ITEM = QTreeWidgetItem.UserType + 26

# library child items
IMPORTS_ITEM = QTreeWidgetItem.UserType + 101
IMPORT_ITEM = QTreeWidgetItem.UserType + 102

PUBLISHERS_ITEM = QTreeWidgetItem.UserType + 103
PUBLISHER_ITEM = QTreeWidgetItem.UserType + 104

SUBSCRIBERS_ITEM = QTreeWidgetItem.UserType + 105
SUBSCRIBER_ITEM = QTreeWidgetItem.UserType + 106

SERVICE_SERVERS_ITEM = QTreeWidgetItem.UserType + 107
SERVICE_SERVER_ITEM = QTreeWidgetItem.UserType + 108

SERVICE_CLIENTS_ITEM = QTreeWidgetItem.UserType + 109
SERVICE_CLIENT_ITEM = QTreeWidgetItem.UserType + 110

ACTION_SERVERS_ITEM = QTreeWidgetItem.UserType + 111
ACTION_SERVER_ITEM = QTreeWidgetItem.UserType + 112

ACTION_CLIENTS_ITEM = QTreeWidgetItem.UserType + 113
ACTION_CLIENT_ITEM = QTreeWidgetItem.UserType + 114

TRANSFORMATIONS_ITEM = QTreeWidgetItem.UserType + 115
TRANSFORMATION_ITEM = QTreeWidgetItem.UserType + 116

STATE_MACHINES_ITEM = QTreeWidgetItem.UserType + 120
STATE_MACHINE_ITEM = QTreeWidgetItem.UserType + 121
MACHINE_STATES_ITEM = QTreeWidgetItem.UserType + 122
MACHINE_STATE_ITEM = QTreeWidgetItem.UserType + 123
STATE_TRANSITIONS_ITEM = QTreeWidgetItem.UserType + 124
STATE_TRANSITION_ITEM = QTreeWidgetItem.UserType + 125

FUNCTIONS_ITEM = QTreeWidgetItem.UserType + 151
FUNCTION_ITEM = QTreeWidgetItem.UserType + 152

FUNCTION_ARGUMENT_ITEM = QTreeWidgetItem.UserType + 200

ROS_LAUNCH_FILE_ITEM = QTreeWidgetItem.UserType + 300
ROS_LAUNCH_PARAM_ITEM = QTreeWidgetItem.UserType + 301
ROS_LAUNCH_INCLUDE_ITEM = QTreeWidgetItem.UserType + 302
ROS_LAUNCH_NODE_ITEM = QTreeWidgetItem.UserType + 310
ROS_LAUNCH_NODE_PARAM_ITEM = QTreeWidgetItem.UserType + 311
ROS_LAUNCH_NODE_REMAP_ITEM = QTreeWidgetItem.UserType + 312

ROCON_LAUNCH_FILE_ITEM = QTreeWidgetItem.UserType + 400


KEY_VALUE_ITEM = QTreeWidgetItem.UserType + 1000

# ROS_COMM_ITEMS = ['pub', 'sub', 'ss', 'sc', 'as', 'ac', 'broadcast', 'listen']
# ROS_PRIMITIVES = ['import', 'param'] + ROS_COMM_ITEMS