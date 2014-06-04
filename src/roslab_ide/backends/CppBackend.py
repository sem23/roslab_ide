__author__ = 'privat'

from roslab_ide.generators.Generator import Generator


class CppClass(object):

    def __init__(self):
        prv_methods = []
        prv_vars = []
        pub_methods = []
        pub_vars = []
        ctor = []
        dtor = []


class ROSLabLibrary(object):

    def __init__(self, name):
        self._name = name
        self._has_run = False

        self._imports = []
        self._publishers = []
        self._subscribers = []
        self._service_clients = []
        self._service_servers = []
        self._tf_broadcasters = []
        self._tf_listeners = []


class HeaderGenerator(Generator):

    def __init__(self, data):
        Generator.__init__(self)
        self._data = data

    def generate_intro(self):
        intro = [
            '#ifndef __{0}_H__'.format(self._data['name'].upper()),
            '#define __{0}_H__'.format(self._data['name'].upper()),
            ''
        ]
        return intro

    def generate_includes(self):
        includes = []
        if 'import' not in self._data:
            return includes
        for include in self._data['import']:
            # 'module' means path/folder, 'include' means include file ;-)
            includes.append('#include {}'.format(include['include']))
        return includes

    def generate_class_intro(self):
        class_intro = [
            'Class {}():'.format(self._data['name']),
            '{',
            'public:'
            '  /* constructor */',
            '  {}();'.format(self._data['name']),
            '  /* destructor */',
            '  ~{}();'.format(self._data['name']),
            '  /* run */',
            '  void run();',
            'private:'
        ]
        return class_intro

    def generate_methods(self):
        methods = []
        if 'functions' not in self._data:
            return methods
        for function in self._data['functions']:
            args = []
            for arg in function['args']:
                args.append(arg['name'])
            methods.append('  {0} {1}({2});'.format(function['return_type'], function['name'], ', '.join(args)))
        return methods

    def generate_vars(self):
        variables = []
        # get parameters
        if 'param' in self._data:
            for parameter in self._data['param']:
                variables.append('  {} {}_;'.format(parameter['type'], parameter['name']))
        if 'comm' in self._data:
            for key, value in self._data['comm']:
                if key == 'pub':
                    cpp_type = 'ros::Publisher'
                    for comm in value:
                        variables.append('  {0} *{1}_{2}_;'.format(cpp_type, comm['topic'], key))
                if key == 'sub':
                    cpp_type = 'ros::Subscriber'
                    for comm in value:
                        variables.append('  {0} *{1}_{2}_;'.format(cpp_type, comm['topic'], key))
                if key == 'ss':
                    cpp_type = 'ros::ServiceServer'
                    for comm in value:
                        variables.append('  {0} *{1}_{2}_;'.format(cpp_type, comm['topic'], key))
                if key == 'sc':
                    cpp_type = 'ros::ServiceClient'
                    for comm in value:
                        variables.append('  {0} *{1}_{2}_;'.format(cpp_type, comm['topic'], key))
        return variables

    def generate_class_outro(self):
        class_outro = [
            '}'
        ]
        return class_outro

    def generate_outro(self):
        outro = [
            '#endif /* #define __{0}_H__ */'.format(self._data['name'].upper())
        ]
        return outro



class SourceGenerator(Generator, CppClass):

    def __init__(self):
        Generator.__init__(self)
        CppClass.__init__(self)


class CppBackend(ROSLabLibrary):

    def __init__(self, name, data):
        ROSLabLibrary.__init__(self, name)
        self.src_gen = SourceGenerator()
        self.hdr_gen = HeaderGenerator()

    def generate_header(self):
        self.hdr_gen.generate()

