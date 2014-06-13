__author__ = 'privat'

import xml.etree.ElementTree as ET
import xml.dom.minidom as dom


class PackageXmlGenerator(object):

    def __init__(self, name, version='0.0.0', description=None, maintainer=('TODO', 'todo@todo.com'), license='TODO',
                 build_depends=None, run_depends=None):

        self._root_element = ET.Element('package')
        name_element = ET.Element('name')
        name_element.text = name
        self._root_element.append(name_element)
        version_element = ET.Element('version')
        version_element.text = version
        self._root_element.append(version_element)
        description_element = ET.Element('description')
        description_element.text = description
        self._root_element.append(description_element)
        maintainer_element = ET.Element('maintainer')
        maintainer_element.text = maintainer[0]
        maintainer_element.set('email', maintainer[1])
        self._root_element.append(maintainer_element)
        license_element = ET.Element('license')
        license_element.text = license
        self._root_element.append(license_element)
        buildtool_element = ET.Element('buildtool_depend')
        buildtool_element.text = 'catkin'
        self._root_element.append(buildtool_element)

        if build_depends:
            for depend in build_depends:
                depend_element = ET.Element('build_depend')
                depend_element.text = depend['name']
                self._root_element.append(depend_element)

        if run_depends:
            for depend in run_depends:
                depend_element = ET.Element('run_depend')
                depend_element.text = depend['name']
                self._root_element.append(depend_element)

    def generate(self):
        tree = ET.tostring(self._root_element, 'utf-8')
        reparsed = dom.parseString(tree)
        return reparsed.toprettyxml(indent='  ')
