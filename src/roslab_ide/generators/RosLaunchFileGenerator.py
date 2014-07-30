__author__ = 'privat'

import xml.etree.ElementTree as ET
import xml.dom.minidom as dom


class RosLaunchFileGenerator(object):

    def __init__(self, data):
        # vars
        self._data = data

    def generate(self):
        root_element = ET.Element('launch')
        
        if 'args' in self._data:
            for arg in self._data['args']:
                arg_element = ET.Element('arg')
                arg_element.set('name', arg['name'])
                arg_element.set('default', arg['default'])
                root_element.append(arg_element)

        if 'params' in self._data:
            for param in self._data['params']:
                param_element = ET.Element('param')
                param_element.set('name', param['name'])
                param_element.set('value', param['value'])
                root_element.append(param_element)

        if 'includes' in self._data:
            for include in self._data['includes']:
                file_attr = '$(find {0})/launch/{1}.launch'.format(include['package'], include['file'])
                include_element = ET.Element('include')
                include_element.set('file', file_attr)
                root_element.append(include_element)

                if 'args' in include:
                    for arg in include['args']:
                        arg_element = ET.Element('arg')
                        arg_element.set('name', arg['name'])
                        arg_element.set('default', arg['default'])
                        include_element.append(arg_element)

        if 'nodes' in self._data:
            for node in self._data['nodes']:
                node_element = ET.Element('node')
                node_element.set('pkg', node['package'])
                node_element.set('type', node['node'])
                node_element.set('name', node['name'])
                node_element.set('output', node['output'])
                if 'machine' in node:
                    node_element.set('machine', node['machine'])
                root_element.append(node_element)

                if 'params' in node:
                    for param in node['params']:
                        param_element = ET.Element('param')
                        param_element.set('name', param['name'])
                        param_element.set('value', param['value'])
                        node_element.append(param_element)

                if 'remaps' in node:
                    for remap in node['remaps']:
                        remap_element = ET.Element('remap')
                        remap_element.set('from', remap['from'])
                        remap_element.set('to', remap['to'])
                        node_element.append(remap_element)

        if 'machines' in self._data:
            for machine in self._data['machines']:
                machine_element = ET.Element('machine')
                machine_element.set('name', machine['name'])
                machine_element.set('address', machine['address'])
                machine_element.set('env-loader', machine['env-loader'])
                machine_element.set('user', machine['user'])
                root_element.append(machine_element)

        # write
        tree = ET.tostring(root_element, 'utf-8')
        reparsed = dom.parseString(tree)
        return reparsed.toprettyxml(indent='  ')