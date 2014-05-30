__author__ = 'privat'


# std imports
import io
import os
import sys
import yaml

# pyqt imports
from python_qt_binding import loadUi
from PyQt4.QtCore import pyqtSlot
from PyQt4.QtGui import QDialog


# ROS imports
import rosmsg
import rospkg
rp = rospkg.RosPack()

# roslab
from roslab_ide.helper.globals import ROS_MSGS, ROS_SRVS, literal_str, check_topic, clean_topic


class CommunicationDialog(QDialog):

    def __init__(self, comm='pub', data=None, parent=None):
        QDialog.__init__(self, parent=parent)
        # setup user interface
        ui_file = os.path.join(rp.get_path('roslab_ide'), 'resource', 'dialogs', 'CommunicationDialog.ui')
        self.ui = loadUi(ui_file, self)

        # signals
        self.ui.commComboBox.currentIndexChanged.connect(self.comm_changed)
        self.ui.packageComboBox.currentIndexChanged.connect(self.package_changed)

        # setup packages
        self.ui.packageComboBox.addItems(ROS_MSGS)

        # data
        if data:
            if comm == 'pub':
                self.ui.commComboBox.setCurrentIndex(0)
            elif comm == 'sub':
                self.ui.commComboBox.setCurrentIndex(1)
            elif comm == 'ss':
                self.ui.commComboBox.setCurrentIndex(2)
            elif comm == 'sc':
                self.ui.commComboBox.setCurrentIndex(3)
            self.ui.topicLineEdit.setText(data['topic'])
            # TODO: set message type
        self.comm = comm
        self.data = {}
        self.callback_data = None

    def accept(self):
        # get communication info
        comm = str(self.ui.commComboBox.currentText())
        topic = str(self.ui.topicLineEdit.text())
        msg_type = str(self.ui.typeComboBox.currentText())
        cleaned_topic = clean_topic(topic)

        # fill communication data
        self.data['msg_type'] = msg_type
        self.data['topic'] = check_topic(topic)

        if comm == 'Publisher':
            self.comm = 'pub'
        elif comm == 'Subscriber':
            self.comm = 'sub'
            # add callback name for subscriber
            suffix = 'msg'
            callback = '{0}_callback'.format(cleaned_topic)
            self.data['callback'] = callback
            self.callback_data = {
                'name': callback,
                'args': [{'name': '{0}_{1}'.format(cleaned_topic, suffix)}],
                'code': '# TODO implement me!'
            }
        elif comm == 'Service Server':
            self.comm = 'ss'
            suffix = 'req'
            # add callback name for subscriber
            callback = '{0}_callback'.format(cleaned_topic)
            self.data['callback'] = callback
            self.callback_data = {
                'name': callback,
                'args': [{'name': '{0}_{1}'.format(cleaned_topic, suffix)}],
                'code': '# TODO implement me!'
            }
        else:
            self.comm = 'sc'
        # accept dialog
        QDialog.accept(self)

    def comm_changed(self):
        # check if we have messages or services
        comm = str(self.ui.commComboBox.currentText())
        if comm in ['Publisher', 'Subscriber']:
            self.ui.packageComboBox.addItems(ROS_MSGS)
        else:
            self.ui.packageComboBox.addItems(ROS_SRVS)

    def package_changed(self):
        # display messages or services from selected package
        package = str(self.ui.packageComboBox.currentText())
        comm = str(self.ui.commComboBox.currentText())
        self.ui.typeComboBox.clear()
        if comm in ['Publisher', 'Subscriber']:
            self.ui.typeComboBox.addItems(rosmsg.list_msgs(package))
        else:
            self.ui.typeComboBox.addItems(rosmsg.list_srvs(package))
