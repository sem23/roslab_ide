from __future__ import print_function
__author__ = 'Peter Rudolph'
__copyright__ = 'Copyright 2014, Germany'
__credits__ = ['Jeanne Dark']
__license__ = 'BSD'
__version__ = '0.0.1'
__maintainer__ = 'Peter Rudolph'
__email__ = 'semael23@gmail.com'
__status__ = 'work in progress'
__info__ = [
    "--- ROSLab IDE ---",
    "Workflow from scratch:",
    "- create new package",
    "- add dependencies to package",
    "- add node(s) to package",
    "- add import(s) to node(s)",
]


# std imports
import os
import sys
import yaml
from yaml.representer import SafeRepresenter
from socket import gethostname

# ROS imports
import rosmsg
import rospkg
rp = rospkg.RosPack()

# pyqt imports
from python_qt_binding import loadUi
from PyQt4.QtGui import QMainWindow, QGridLayout, QIcon, QMessageBox, QLabel, QImage, QPixmap
from PyQt4.QtGui import QApplication, QCursor, QWidgetAction
from PyQt4.QtCore import Qt, QSettings, QSize, QPoint, pyqtSlot

# roslab imports
import roslab_ide.helper.globals as g

from roslab_ide.dialogs.SettingsDialog import SettingsDialog
from roslab_ide.helper.ROSCommand import ROSCommand
from roslab_ide.helper.OutputWriter import OutputWriter
from roslab_ide.helper.Workspace import Controller
from roslab_ide.backends.PyEditor import PyEditor, PyFunctionEditor


def change_style(style, representer):
    def new_representer(dumper, data):
        scalar = representer(dumper, data)
        scalar.style = style
        return scalar
    return new_representer

represent_literal_str = change_style('|', SafeRepresenter.represent_str)
yaml.add_representer(g.literal_str, represent_literal_str)


class MainWindow(QMainWindow):

    def __init__(self):
        QMainWindow.__init__(self)
        # setup user interface
        ui_file = os.path.join(rp.get_path('roslab_ide'), 'resource', 'ROSLab.ui')
        self.ui = loadUi(ui_file, self)
        self.ui.editorTabWidget.tabCloseRequested.connect(self.close_editor_tab)
        self.ui.previewTextEdit = PyEditor()
        QGridLayout(self.ui.backendDockWidgetContents).addWidget(self.ui.previewTextEdit)
        self.ui.workspaceTreeWidget.setSortingEnabled(True)
        self.ui.workspaceTreeWidget.sortItems(0, Qt.AscendingOrder)
        ros_env = g.get_ros_env()
        self.ui.masterLineEdit.setText(ros_env['ROS_MASTER_URI'])
        self.ui.hostLineEdit.setText(ros_env['ROS_HOSTNAME'])

        # additional icons
        self.ui.actionSettings.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'tool_properties.png')))
        self.ui.actionStart_roscore.setIcon(QIcon(os.path.join(
            rp.get_path('roslab_ide'), 'resource', 'icons', 'konsole1.png')))

        # vars
        self._item_menu_action = None

        # setup standard out
        sys.stdout = OutputWriter(self.ui.standardOutTextEdit)

        # setup globals
        g.main_widget = self
        g.preview_widget = self.ui.previewTextEdit
        g.ROS_PKGS = sorted(rp.list())

        # workspace
        self._workspace_controller = Controller(root_item=self.ui.workspaceTreeWidget.invisibleRootItem(),
                                                parent_widget=self)
        # signals
        self.ui.masterLineEdit.textChanged.connect(self.change_master_uri)
        self.ui.hostLineEdit.textChanged.connect(self.change_hostname)
        self.ui.zeroconfPushButton.clicked.connect(self.set_zeroconf)
        self.ui.workspaceTreeWidget.itemSelectionChanged.connect(self.tree_item_selected)
        self.ui.workspaceTreeWidget.customContextMenuRequested.connect(self.show_workspace_item_context_menu)
        self.ui.actionSettings.triggered.connect(self.show_ide_settings)
        self.ui.actionStart_roscore.triggered.connect(ROSCommand.roscore)
        self.ui.actionStart_rviz.triggered.connect(ROSCommand.rviz)
        self.ui.actionStart_rqt.triggered.connect(ROSCommand.rqt)

        # actions (dock widgets visibility)
        action = self.ui.stdOutDockWidget.toggleViewAction()
        action.setText('std out')
        action.setIcon(QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'konsole1.png')))
        self.ui.menuWindows.addAction(action)
        action = self.ui.backendDockWidget.toggleViewAction()
        action.setText('backend preview')
        action.setIcon(QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'readme.png')))
        self.ui.menuWindows.addAction(action)
        action = self.ui.machineDockWidget.toggleViewAction()
        action.setText('fsm preview')
        action.setIcon(QIcon(os.path.join(rp.get_path('roslab_ide'), 'resource', 'icons', 'state_machine.png')))
        self.ui.menuWindows.addAction(action)

        # restore global settings
        g.restore_settings()
        # restore window settings
        self.load_settings()

    def show_ide_settings(self):
        settings_dialog = SettingsDialog(parent=self)
        settings_dialog.exec_()

    def tree_item_selected(self):
        current_item = self.ui.workspaceTreeWidget.currentItem()
        # check if selected item is library item
        item_type = current_item.type()
        if item_type == g.LIBRARY_ITEM:
            # generate preview
            current_item.preview()
            if not g.keep_functions_open:
                for index in range(self.ui.editorTabWidget.count()):
                    widget = self.ui.editorTabWidget.widget(0)
                    self.ui.editorTabWidget.removeTab(0)
                    del widget
            # open library functions in editor
            for function_item in current_item.function_items():
                package = current_item.package_name()
                library = current_item.name()
                function = function_item.name()
                args = function_item.args()
                # TODO: make tab widgets tab bar nicer!
                window_title = (
                    'Library: {}\n'.format(library) +
                    'Function: {}\n'.format(function) +
                    'Arguments: {}'.format(', '.join(args))
                )
                window_title_exists = False
                # check if function is already opened
                if not g.keep_functions_open:
                    for page in range(self.ui.editorTabWidget.count()):
                        if str(self.ui.editorTabWidget.tabText(page)) == window_title:
                            window_title_exists = True
                if not window_title_exists:
                    function_data = Controller.get_function_data(package, library, function)
                    editor = PyFunctionEditor(function_data=function_data, package=package,
                                              library=library, parent=self)
                    self.ui.editorTabWidget.addTab(editor, QIcon(os.path.join(
                        rp.get_path('roslab_ide'), 'resource', 'icons', 'function.png')), window_title)

        if item_type == g.STATE_MACHINE_ITEM:
            package = current_item.package_name()
            library = current_item.library_name()
            machine = current_item.name()
            Controller.visualize_state_machine(package, library, machine)
            window_title = 'Library: {0} | Machine: {1}'.format(library, machine)
            graph_png = QImage('/tmp/fsm.png')
            self.ui.fsmGraphLabel.setPixmap(QPixmap.fromImage(graph_png))
            self.ui.machineDockWidget.setWindowTitle(window_title)

    def close_editor_tab(self, index):
        tab_widget = self.ui.editorTabWidget.widget(index)
        self.ui.editorTabWidget.removeTab(index)
        del tab_widget

    def show_workspace_item_context_menu(self):
        current_item = self.ui.workspaceTreeWidget.currentItem()
        if not current_item:
            return
        context_menu = self.ui.workspaceTreeWidget.currentItem().get_context_menu()
        if not context_menu:
            return
        context_menu.exec_(QCursor.pos())

    @pyqtSlot()
    def set_zeroconf(self):
        hostname = gethostname()
        self.ui.masterLineEdit.setText('http://' + hostname + '.local:11311')
        self.ui.hostLineEdit.setText(hostname + '.local')

    @pyqtSlot()
    def change_master_uri(self):
        os.environ['ROS_MASTER_URI'] = str(self.ui.masterLineEdit.text())

    @pyqtSlot()
    def change_hostname(self):
        os.environ['ROS_HOSTNAME'] = str(self.ui.hostLineEdit.text())

    def closeEvent(self, event):
        really_quit = QMessageBox.question(self, 'Close ROSLab IDE', 'Do you really want to close application?',
                                           QMessageBox.No, QMessageBox.Yes)
        if really_quit == QMessageBox.Yes:
            # store widget settings
            self.save_settings()
            # store globals (MSGS, SRVS)
            g.store_settings()
            # store workspace settings
            Controller.store_settings()
            if Controller.has_unsaved_changes():
                save_changes = QMessageBox.question(self, 'Close ROSLab IDE', 'Do you want to save changes?',
                                                    QMessageBox.No, QMessageBox.Yes)
                if save_changes == QMessageBox.Yes:
                    Controller.save_workspace()
            event.accept()
        else:
            event.ignore()

    def save_settings(self):
        settings = QSettings('semCo', 'ROSLab IDE')
        # store window settings
        settings.beginGroup('MainWindow')
        settings.setValue('geometry', self.saveGeometry())
        settings.setValue('state', self.saveState())
        settings.endGroup()

    def load_settings(self):
        settings = QSettings('semCo', 'ROSLab IDE')
        # restore window settings
        settings.beginGroup('MainWindow')
        geometry = settings.value('geometry')
        if geometry:
            self.restoreGeometry(geometry)
        state = settings.value('state')
        if state:
            self.restoreState(state)
        settings.endGroup()


class Application(QApplication):
    def __init__(self, args):
        QApplication.__init__(self, args)
        # show main window
        self.main_window = MainWindow()
        self.main_window.show()