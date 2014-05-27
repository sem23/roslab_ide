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

# ROS imports
import rosmsg
import rospkg
rp = rospkg.RosPack()

# pyqt imports
from python_qt_binding import loadUi
from PyQt4.QtGui import QMainWindow, QGridLayout, QTreeWidgetItem, QProgressDialog, QMenu, QMessageBox
from PyQt4.QtGui import QApplication, QCursor
from PyQt4.QtCore import Qt, QSettings, QSize, QPoint

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
        self.ui.previewTextEdit = PyEditor()
        self.ui.editorLayout = QGridLayout(self.ui.editorFrame).addWidget(self.ui.previewTextEdit)

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
        self.ui.workspaceTreeWidget.itemSelectionChanged.connect(self.tree_item_selected)
        self.ui.workspaceTreeWidget.customContextMenuRequested.connect(self.show_workspace_item_context_menu)
        self.ui.actionSettings.triggered.connect(self.show_ide_settings)
        self.ui.actionStart_roscore.triggered.connect(ROSCommand.roscore)
        # signals (widgets visibility)
        self.ui.actionBackend_Out.triggered.connect(self.ui.editorFrame.setVisible)
        self.ui.actionStandard_Out.triggered.connect(self.ui.standardOutTextEdit.setVisible)

        # restore global settings
        g.restore_settings()
        # restore window settings
        self.load_settings()

    def show_ide_settings(self):
        settings_dialog = SettingsDialog(parent=self)
        settings_dialog.exec_()

    def tree_item_selected(self):
        if self._item_menu_action:
            self.ui.menubar.removeAction(self._item_menu_action)
            self._item_menu_action = None
        current_item = self.ui.workspaceTreeWidget.currentItem()
        # display item context menu in menu bar
        context_menu = current_item.get_context_menu()
        if context_menu:
            self._item_menu_action = self.ui.menubar.addMenu(context_menu)
        # check if selected item is library item
        item_type = current_item.type()
        if item_type == g.LIBRARY_ITEM:
            # generate preview
            current_item.preview()
            # open library functions in editor
            for function_item in current_item.function_items():
                package = current_item.package_name()
                library = current_item.name()
                function = function_item.name()
                window_title = '{0} - {1}'.format(library, function)
                window_title_exists = False
                # check if function is already opened
                for page in range(self.ui.editorTabWidget.count()):
                    if str(self.ui.editorTabWidget.tabText(page)) == window_title:
                        window_title_exists = True
                if not window_title_exists:
                    function_data = Controller.get_function_data(package, library, function)
                    editor = PyFunctionEditor(function_data=function_data, parent=self)
                    self.ui.editorTabWidget.addTab(editor, window_title)

    def show_workspace_item_context_menu(self):
        current_item = self.ui.workspaceTreeWidget.currentItem()
        if not current_item:
            return
        context_menu = self.ui.workspaceTreeWidget.currentItem().get_context_menu()
        if not context_menu:
            return
        context_menu.exec_(QCursor.pos())

    def closeEvent(self, event):
        really_quit = QMessageBox.question(self, 'Close ROSLab IDE', 'Do you really want to close application?',
                                           QMessageBox.No, QMessageBox.Yes)
        if really_quit == QMessageBox.Yes:
            self.save_settings()
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
        settings.setValue('pos', self.pos())
        settings.setValue('size', self.size())
        settings.setValue('maximized', self.isMaximized())
        settings.endGroup()
        settings.beginGroup('Backend Out')
        settings.setValue('visible', self.ui.editorFrame.isVisible())
        settings.endGroup()
        settings.beginGroup('Standard Out')
        settings.setValue('visible', self.ui.standardOutTextEdit.isVisible())
        settings.endGroup()

    def load_settings(self):
        settings = QSettings('semCo', 'ROSLab IDE')
        # restore window settings
        settings.beginGroup('MainWindow')
        self.resize(settings.value('size', QSize(1240, 800)))
        self.move(settings.value('pos', QPoint(0, 0)))
        if str(settings.value('maximized', 'false')) != 'false':
            self.showMaximized()
        settings.endGroup()
        # standard out
        settings.beginGroup('Standard Out')
        if str(settings.value('visible', 'false')) == 'false':
            self.ui.standardOutTextEdit.setVisible(False)
            self.ui.actionStandard_Out.setChecked(False)
        else:
            self.ui.standardOutTextEdit.setVisible(True)
            self.ui.actionStandard_Out.setChecked(True)
        settings.endGroup()
        # backend out
        settings.beginGroup('Backend Out')
        if str(settings.value('visible', 'false')) == 'false':
            self.ui.editorFrame.setVisible(False)
            self.ui.actionBackend_Out.setChecked(False)
        else:
            self.ui.editorFrame.setVisible(True)
            self.ui.actionBackend_Out.setChecked(True)
        settings.endGroup()


class Application(QApplication):
    def __init__(self, args):
        QApplication.__init__(self, args)
        # show main window
        self.main_window = MainWindow()
        self.main_window.show()