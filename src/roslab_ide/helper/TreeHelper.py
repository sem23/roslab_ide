from pygccxml.declarations.type_traits import array_item_type
from spyderlib.widgets.editortools import item_at_line

__author__ = 'privat'

import os

# pyqt imports
from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtGui import QAction, QMenu, QTreeWidgetItem

import roslab_ide.helper.Workspace as ws

# globals
import roslab_ide.helper.globals as g

# TODO build tree directly from dict
# TODO each special item must have a Constructor initializing its own data, especially its children!




#
#
#
#
# class TreeItem():
#
#     def __init__(self):
#         pass
#
#     @staticmethod
#     def from_current():
#         item = TreeHelper.current_item()
#         name = str(item.text(0))
#         data = None
#
#         item_type = item.type()
#         if item_type == g.ROSLAB_PACKAGE_ITEM:
#             data = Workspace.roslab_packages()[name]
#         elif item_type == g.IMPORT_ITEM:
#             package_name = item.parent().parent().text(0)
#             data = Workspace.roslab_packages()[package_name]['import']
#         elif item_type == g.ROS_PARAM_ITEM:
#             package_name = item.parent().parent().text(0)
#             data = Workspace.roslab_packages()[package_name]['param']
#         elif item_type == g.FUNCTION_ITEM:
#             package_name = item.parent().parent().text(0)
#             data = Workspace.roslab_packages()[package_name]['function']
#         return name, item, data
#
#     @staticmethod
#     def remove():
#         raise RuntimeError('Function "remove" MUST be re-implemented!')


class TreeHelper():

    _tree = None

    def __init__(self, tree):
        TreeHelper._tree = tree

    @staticmethod
    def get_package_item(item):
        item_type = item.type()
        if QTreeWidgetItem.UserType + 20 < item_type < QTreeWidgetItem.UserType + 30:
            return item.parent().parent()

    @staticmethod
    def get_library_item(item):
        item_type = item.type()
        if item_type == g.ROS_COMM_ITEM:
            return item.parent().parent()

    @staticmethod
    def get_child_item_by_text(item, child_text):
        for i in range(item.childCount()):
            if str(item.child(i).text(0)) == child_text:
                return item.child(i)
        return None

    @staticmethod
    def fill_item_from_dict(item, key, dict_, parent_key=None):
        if key == 'package':
            child_item = item
            parent_key = key
        elif key is None:
            child_item = item
        else:
            # determine item type from key and/or parent key
            if key == 'package':
                item_type = g.PACKAGE_ITEM
            elif key == 'function':
                item_type = g.FUNCTION_ITEM
            elif parent_key == 'libraries':
                item_type = g.LIBRARY_ITEM
            else:
                item_type = g.UNDEFINED_ITEM
            # create child item
            child_item = QTreeWidgetItem(item, [key], type=item_type)
            # setup next parent key
            if key in ['libraries', 'nodes']:
                parent_key = key
            else:
                parent_key = None
        for k, v in dict_.iteritems():
            TreeHelper.fill_item_from_value(child_item, k, v, parent_key)

    @staticmethod
    def fill_item_from_list(item, key, list_, parent_key=None):
        list_item = QTreeWidgetItem([key])
        for element in list_:
            if type(element) is dict:
                TreeHelper.fill_item_from_dict(item, key, element, parent_key)
            elif type(element) is list:
                TreeHelper.fill_item_from_list(item, key, element, parent_key)
            else:
                QTreeWidgetItem(list_item, [element])
        if list_item.childCount():
            item.addChild(list_item)

    @staticmethod
    def fill_item_from_value(item, key, value, parent_key=None):
        # print key, parent_key
        if type(value) is dict:
            TreeHelper.fill_item_from_dict(item, key, value, parent_key)
        elif type(value) is list:
            TreeHelper.fill_item_from_list(item, key, value, parent_key)
        else:
            if parent_key == 'nodes':
                item_type = g.NODE_ITEM
            else:
                item_type = g.UNDEFINED_ITEM
            child_item = QTreeWidgetItem(item, [key], item_type)
            child_item.setText(1, str(value))

    @staticmethod
    def update_item(item, data):
        item_name = str(item.text(0))
        item_type = item.type()
        parent_item = item.parent()
        if not parent_item:
            raise RuntimeError("To update a tree item it MUST have a parent!")
        parent_item.removeChild(item)
        item = QTreeWidgetItem([item_name], type=item_type)
        parent_item.addChild(item)
        # add updated child data
        TreeHelper.fill_item_from_dict(item, None, data)

    @staticmethod
    def current_item():
        return TreeHelper._tree.currentItem()