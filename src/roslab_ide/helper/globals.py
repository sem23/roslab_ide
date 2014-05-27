__author__ = 'privat'

import yaml

import rosmsg

# pyqt imports
from PyQt4.QtGui import QProgressDialog, QTreeWidgetItem, QMessageBox
from PyQt4.QtCore import Qt, QSettings

# settings
name = 'Change name in global settings'
author = 'Change name in author settings'
email = 'Change name in email settings'
place = 'Change name in place settings'
license_ = 'Change name in license settings'

# generator backends
backends = ['python']

# general
main_widget = None
preview_widget = None

# messages & services
ROS_PKGS = []
ROS_MSGS = []
ROS_SRVS = []


# make nice literal text while yaml dump
class literal_str(str):
    pass


def get_dict_list_entry_by_key_value(list_, key, value):
    if type(list_) is not list:
        raise TypeError('Given list is not from type list!')
    for entry in list_:
        if entry[key] is value:
            return entry
    raise TypeError('Entry not found! Check if you are looking in correct list!')


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
    for package in ROS_PKGS:
        if progress_dialog.wasCanceled():
            ROS_MSGS = []
            ROS_SRVS = []
            break
        progress_dialog.setValue(count)
        progress_dialog.setLabelText('Scanning ROS Distro\nscanning {}...'.format(package))
        if len(rosmsg.list_msgs(package)) > 0:
            ROS_MSGS.append(package)
            progress_dialog.setLabelText('Scanning ROS Distro\nscanning {}...\nfound messages...'.format(package))
        if len(rosmsg.list_srvs(package)) > 0:
            progress_dialog.setLabelText('Scanning ROS Distro\nscanning {}...\nfound services...'.format(package))
            ROS_SRVS.append(package)
        count += 1
    progress_dialog.setValue(len(ROS_PKGS))


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
    print('found {0} packages containing messages'.format(len(ROS_MSGS)))
    # get service packages
    size = settings.beginReadArray('srv_packages')
    for i in range(size):
        settings.setArrayIndex(i)
        name = settings.value('name', '')
        ROS_SRVS.append(name)
    settings.endArray()
    print('found {0} packages containing services'.format(len(ROS_SRVS)))
    if len(ROS_MSGS) == 0 and len(ROS_SRVS) == 0:
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

FUNCTIONS_ITEM = QTreeWidgetItem.UserType + 115
FUNCTION_ITEM = QTreeWidgetItem.UserType + 116

TRANSFORMATIONS_ITEM = QTreeWidgetItem.UserType + 117
TRANSFORMATION_ITEM = QTreeWidgetItem.UserType + 118

FUNCTION_ARGUMENT_ITEM = QTreeWidgetItem.UserType + 200

KEY_VALUE_ITEM = QTreeWidgetItem.UserType + 300

# ROS_COMM_ITEMS = ['pub', 'sub', 'ss', 'sc', 'as', 'ac', 'broadcast', 'listen']
# ROS_PRIMITIVES = ['import', 'param'] + ROS_COMM_ITEMS