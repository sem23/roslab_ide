from __future__ import print_function

__author__ = 'privat'

import pygraphviz
import collections

from operator import itemgetter
from roslab_ide.helper.globals import clean_topic

## frontend - description
# import:
# - class: 'name'                                       # python allows '*' as class
#   module: 'name'                                      # required for python, optional for c++
# param:
# - name: 'name'
#   default: 'value'
#   type: 'type'                                        # required for c++
# comm:
#   pub / sub / ssr / sc / sas / sac (/ as / ac):       # lists
#   - topic: 'name'
#     msg_type: package/type                            # also srv types
#     cb: 'name'                                        # required for sub, ss
#     execute_cb: 'name'                                # required for s-as
#     result_cb: 'name'                                 # required for s-ac
#     feedback_cb: 'name'                               # optional for s-ac
# tf:
#   broadcast:                                          # lists
#   - parent: 'parent_frame'
#     child: 'child_frame'
#   listen:                                             # lists
#   - parent: 'parent_frame'
#     child: 'child_frame'
# function:
# - name: str
#   args:
#   - name: 'name'
#     type: 'type'                                      # required for c++
#     default: 'value'                                  # optional
#   return: 'type'                                      # required for c++
#   code: |
#     'source code'


class PyBackend():

    def __init__(self, name, data):
        self._name = name
        self._data = data
        self._bases = 'object'
        self._has_run = False

        self._imports = []
        self._publishers = []
        self._subscribers = []
        self._service_clients = []
        self._service_servers = []
        self._tf_broadcasters = []
        self._tf_listeners = []

        self._intro = []
        self._class_initials = []
        self._static_class_initials = []
        self._param_init = []
        self._pub_init = []
        self._sub_init = []
        self._ss_init = []
        self._sc_init = []
        self._as_init = []
        self._ac_init = []
        self._tfl_init = []
        self._tfb_init = []
        self._functions = []
        self._add_init = []

        self._fsm_init = ''
        self._fsm_states = []
        self._fsm_start_state = ''

        self._run_function = []
        self._run_rate = None

        self.load_from_data(data)

    def parse_imports(self, imports):
        """
        Parse imports.
        """
        if type(imports) is not list:
            raise TypeError('imports must be a "list"')
        for import_ in imports:
            self.add_import(**import_)

    def parse_parameters(self, parameters):
        """
        Parse parameters.
        """
        if type(parameters) is not list:
            raise TypeError('parameters must be a "list"')
        for parameter in parameters:
            self.add_parameter(**parameter)

    def parse_communications(self, communications):
        """
        Parse communications in form {comm: [topic, type, ...], ...}.
        """
        if type(communications) is not dict:
            raise TypeError('communications must be a "dict"')
        for comm, comms in communications.iteritems():
            if comm == 'pub':
                for publisher in comms:
                    self.add_publisher(**publisher)
            elif comm == 'sub':
                for subscriber in comms:
                    self.add_subscriber(**subscriber)
            elif comm == 'ss':
                for service_server in comms:
                    self.add_service_server(**service_server)
            elif comm == 'sc':
                for service_client in comms:
                    self.add_service_client(**service_client)
            elif comm == 'sas':
                for simple_action_server in comms:
                    self.add_simple_action_server(**simple_action_server)
            elif comm == 'sac':
                for simple_action_client in comms:
                    self.add_simple_action_client(**simple_action_client)
            elif comm == 'as':
                for action_server in comms:
                    self.add_action_server(**action_server)
            elif comm == 'ac':
                for action_client in comms:
                    self.add_action_client(**action_client)

    def parse_transforms(self, transforms):
        """
        Parse transforms in form {comm: [topic, type, ...], ...}.
        """
        if type(transforms) is not dict:
            raise TypeError('transforms must be a "dict"')
        for tf_type, tfs in transforms.iteritems():
            if tf_type == 'broadcaster':
                for tf_broadcaster in tfs:
                    self.add_tf_broadcaster(**tf_broadcaster)
            elif tf_type == 'listener':
                for tf_listener in tfs:
                    self.add_tf_listener(**tf_listener)

    def parse_functions(self, functions):
        """
        Parse functions in form [{name: {args : {name: 'str'} , code : 'str'}].
        """
        if type(functions) is not list:
            raise TypeError('functions must be a "list"')
        for function in functions:
            self.add_function(**function)

    def load_from_data(self, data):
        self.set_intro(
            data['author'],
            data['copyright'],
            data['credits'],
            data['license'],
            data['version'],
            data['maintainer'],
            data['email'],
            data['status']
        )
        # add rospy import
        self.add_import('rospy')
        if 'import' in data:
            self.parse_imports(data['import'])
        if 'param' in data:
            self.parse_parameters(data['param'])
        # comms
        if 'comm' in data:
            self.parse_communications(data['comm'])
        # transforms
        if 'tf' in data:
            self.parse_transforms(data['tf'])
        if 'functions' in data:
            self.parse_functions(data['functions'])
        if 'use_fsm' in data:
            use_fsm = data['use_fsm']
        else:
            use_fsm = None
        if 'run_rate' in data:
            run_rate = data['run_rate']
        else:
            if use_fsm:
                run_rate = 1
            else:
                run_rate = None

        if use_fsm:
            i = 0
            graph = pygraphviz.AGraph(directed=True)
            graph.add_node('done', color='red', shape='house')
            self.add_fsm(run_rate)
            for state in data['states']:
                if 'name' in state:
                    name = state['name']
                else:
                    i += 1
                    name = 'unknown_state_' + i
                if graph.has_node(name):
                    node = graph.get_node(name)
                    node.attr['color'] = 'blue'
                    node.attr['shape'] = 'rect'
                else:
                    graph.add_node(name, color='blue', shape='rect')
                if 'handler' in state:
                    handler = state['handler']
                else:
                    handler = name.replace(' ', '_') + '_state_handler'
                    self.add_fsm_state_handler(
                        'def ' + handler + '(self, cargo):\n'
                        '    # TODO: implement state handler!\n'
                        "    return 'newState', cargo"
                    )
                if 'end_state' in state:
                    end_state = state['end_state']
                else:
                    end_state = False
                if end_state:
                    node = graph.get_node(name)
                    node.attr['color'] = 'red'
                    node.attr['shape'] = 'house'
                if 'trans' in state:
                    for transition in state['trans']:
                        graph.add_edge(name, transition['name'],
                                       label=transition['condition'],
                                       fontsize=10)
                else:
                    # TODO: implement warning for incorrect graph
                    pass
                self.add_fsm_state(name=name, handler=handler, end_state=end_state)
            if 'start_state' in data:
                start_state = data['start_state']
                node = graph.get_node(start_state)
                node.attr['color'] = 'green'
                node.attr['shape'] = 'invhouse'
                self.set_fsm_start_state(start_state)
            # graph.layout()
            graph.layout(prog='dot')
            # graph.layout(prog='sfdp')
            # graph.layout(prog='twopi')
            # graph.layout(prog='circo')
            # graph.layout(prog='fdp')
            graph.draw(self._name.lower() + '_fsm_graph.png')
        else:
            if not self._has_run:
                if run_rate:
                    self._run_function.append('def run(self):')
                    self.add_parameter('run_rate', run_rate)
                    self._run_function.append('r = rospy.Rate(self._run_rate)')
                    self._run_function.append('while not rospy.is_shutdown():')
                    self._run_function.append('    # TODO: implement run function!')
                    self._run_function.append('    r.sleep()')
                else:
                    self._run_function.append('@staticmethod\n    def run():')
                    self._run_function.append('rospy.spin()')

    def add_fsm(self, run_rate=None):
        if self._run_rate is not None:
            raise RuntimeError('Can not add Finite State Machine! Class already has a run function!')
        self.add_import('ros_wrapper_generator', 'FiniteStateMachine')
        self._fsm_init = 'self._fsm = FiniteStateMachine()'
        self.add_fsm_state('done', handler='None', end_state=True)
        if run_rate is not None:
            self._run_rate = run_rate
            self.add_parameter('run_rate', run_rate)
            self._run_function = []
            self._run_function.append('def run(self, cargo=None):')
            self._run_function.append('self._fsm.run(cargo)')

    def add_fsm_state(self, name, handler='None', end_state=False):
        if handler is 'None':
            self._fsm_states.append("self._fsm.add_state('{0}', {1}, end_state={2})".format(
                name, handler, end_state))
        else:
            self._fsm_states.append("self._fsm.add_state('{0}', self.{1}, end_state={2})".format(
                name, handler, end_state))

    def set_fsm_start_state(self, start_state):
        self._fsm_start_state = "self._fsm.set_start('{}')".format(start_state)

    def add_fsm_state_handler(self, content):
        content = content.splitlines()
        self._functions.append(self.make_intended_block(content))

    def add_parameter(self, name, default=None):
        if default is None:
            self._param_init.append("self._{0} = rospy.get_param('{0}')".format(name))
        else:
            if type(default) is str:
                self._param_init.append("self._{0} = rospy.get_param('{0}', default='{1}')".format(name, default))
            else:
                self._param_init.append("self._{0} = rospy.get_param('{0}', default={1})".format(name, default))

    def add_publisher(self, msg_type, topic):
        # get module and class from msg type
        [module, msg_class] = msg_type.split('/')
        # clean topic
        cleaned_topic = clean_topic(topic)
        # add required import
        self.add_import(module + '.msg', msg_class)
        # setup publisher initializer
        self._pub_init.append("self.{2}_pub = rospy.Publisher('{0}', {1}, queue_size=10)".format(
            topic, msg_class, cleaned_topic))
        # add published data to class initials
        self._class_initials.append('self.{0}_msg = {1}()'.format(cleaned_topic, msg_class))

    def add_subscriber(self, msg_type, topic, callback):
        # get module and class from msg type
        [module, msg_class] = msg_type.split('/')
        # clean topic
        cleaned_topic = clean_topic(topic)
        # add required import
        self.add_import(module + '.msg', msg_class)
        # check if callback is set, otherwise construct standard callback
        # if not callback:
        #     cb = '{0}_cb'.format(cleaned_topic)
        #     # setup callback function
        #     cb_function = [
        #         'def {0}_cb(self, {0}):'.format(cleaned_topic),
        #         'self.{0} = {0}'.format(cleaned_topic)
        #     ]
        #     self.add_function(cb_function)
        # setup subscriber initializer
        self._sub_init.append("rospy.Subscriber('{0}', {1}, self.{2}, queue_size=10)".format(
            topic, msg_class, callback))
        # add subscribed data to class initials
        self._class_initials.append('self.{0}_msg = {1}()'.format(cleaned_topic, msg_class))

    def add_service_server(self, msg_type, topic, cb=None):
        # TODO: fixme!
        # get module and class from msg type
        [module, msg_class] = msg_type.split('/')
        # clean topic
        cleaned_topic = clean_topic(topic)
        # add required imports
        self.add_import(module + '.srv', msg_class)
        self.add_import(module + '.srv', msg_class + 'Response')
        # check if callback is set, otherwise construct standard callback
        if not cb:
            cb = '{0}_cb'.format(cleaned_topic)
            cb_function = [
                'def {0}_cb(self, {0}_req):'.format(cleaned_topic),
                '{0}_res = {1}Response()'.format(cleaned_topic),
                'return {0}_res'.format(cleaned_topic)
            ]
            self.add_function(cb_function)
        # setup service server initializer
        self._ss_init.append("self.{3}_ss = rospy.Service('{0}', {1}, self.{2})".format(
            topic, msg_class, cb, cleaned_topic))

    def add_service_client(self, msg_type, topic):
        # TODO: fixme!
        # get module and class from msg type
        [module, msg_class] = msg_type.split('/')
        # clean topic
        cleaned_topic = clean_topic(topic)
        # add required imports
        self.add_import('rospy.exceptions', 'ROSException')
        self.add_import(module + '.srv', msg_class)
        self.add_import(module + '.srv', msg_class + 'Request')
        # add service request to class initials
        self._class_initials.append('self.{0}_req = {1}Request()'.format(cleaned_topic, msg_class))
        # setup service client initializer
        self._sc_init.append("try:")
        self._sc_init.append("    rospy.wait_for_service('{0}', timeout=3)".format(topic))
        self._sc_init.append("except ROSException as e:")
        self._sc_init.append("    print(e.message + '. Make sure that service node is running!')")
        self._sc_init.append("self.{2}_sc = rospy.ServiceProxy('{0}', {1})".format(
            topic, msg_class, cleaned_topic))

    def add_tf_broadcaster(self, parent_frame, child_frame):
        # add required imports
        self.add_import('tf')
        self.add_import('geometry_msgs.msg', 'PoseStamped')
        # setup tf broadcaster initializer
        self._tfb_init.append('self.{0}{1}_tfb = tf.TransformBroadcaster()'.format(
            parent_frame.replace('/', '_'), child_frame.replace('/', '_')))
        # setup send from pose function
        function = [
            'def send{0}{1}_tf(self, pose):'.format(
                parent_frame.replace('/', '_'), child_frame.replace('/', '_')),
            '    translation = pose.pose.position',
            '    rotation = pose.pose.orientation',
            '    self.{0}{1}_tfb.sendTransform('.format(
                parent_frame.replace('/', '_'), child_frame.replace('/', '_')),
            '        (translation.x, translation.y, translation.z),',
            '        (rotation.x, rotation.w, rotation.z, rotation.w),',
            '        rospy.Time.now(),',
            "        '{}',".format(child_frame),
            "        '{}')".format(parent_frame),
        ]
        self._functions.append(self.make_intended_block(function))

    def add_tf_listener(self, parent_frame, child_frame):
        # cleaned frames
        parent = parent_frame.replace('/', '_')
        child = child_frame.replace('/', '_')
        # add required imports
        self.add_import('tf')
        self.add_import('rospy.exceptions', 'ROSException')
        self.add_import('geometry_msgs.msg', 'PoseStamped')
        # setup tf listener initializer
        self._tfl_init.append('self.{0}{1}_tfl = tf.TransformListener()'.format(parent, child))
        # setup get as pose function
        function = [
            'def get{0}{1}_tf(self):'.format(parent, child),
            '    pose = PoseStamped()',
            '    try:',
            "        trans, rot = self.{0}{1}_tfl.lookupTransform('{2}', '{3}', rospy.Time(0))".format(
                parent, child, parent_frame, child_frame),
            '    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):',
            '        # TODO: implement TransformListener.lookupTransform() exception handling!',
            '        return pose',
            '    pose.pose.position.x = trans[0]',
            '    pose.pose.position.y = trans[1]',
            '    pose.pose.position.z = trans[2]',
            '    pose.pose.orientation.x = rot[0]',
            '    pose.pose.orientation.y = rot[1]',
            '    pose.pose.orientation.z = rot[2]',
            '    pose.pose.orientation.w = rot[3]',
            '    return pose'
        ]
        self._functions.append(self.make_intended_block(function))

    def add_simple_action_server(self, msg_type, topic, execute_callback):
        # TODO: fixme!
        # get module and class from msg type
        [module, msg_class] = msg_type.split('/')
        # clean topic
        cleaned_topic = clean_topic(topic)
        # add required imports
        self.add_import('actionlib')
        self.add_import(module + '.msg', msg_class + 'Feedback')
        self.add_import(module + '.msg', msg_class + 'Result')
        self.add_import(module + '.msg', msg_class + 'Action')
        # check if callback is set, otherwise construct standard callback
        if not execute_callback:
            execute_callback = '{0}_execute_cb'.format(cleaned_topic)
            self.add_function(execute_callback, {'self': None, '{}_req'.format(cleaned_topic): None}, [
                '{0}_feedback = {1}Feedback()'.format(cleaned_topic, msg_class),
                '{0}_result = {1}Result()'.format(cleaned_topic, msg_class),
                '# TODO implement me!'
            ])
        # setup simple action server initializer
        self._as_init.append("self.{3}_sas = actionlib.SimpleActionServer('{0}', {1}Action, execute_cb={2})".format(
            topic, msg_class, execute_callback, cleaned_topic))

    def add_simple_action_client(self, msg_type, topic, feedback_callback, result_callback):
        # TODO: fixme!
        # get module and class from msg type
        [module, msg_class] = msg_type.split('/')
        # clean topic
        cleaned_topic = clean_topic(topic)
        # add required imports
        self.add_import('actionlib')
        self.add_import(module + '.msg', msg_class + 'Goal')
        self.add_import(module + '.msg', msg_class + 'Action')

        # setup simple action server initializer
        self._ac_init.append("self.{2}_ac = actionlib.SimpleActionClient('{0}', {1}Action)".format(
            topic, msg_class, cleaned_topic))

    def add_action_server(self, msg_type, topic):
        # TODO implement me!
        pass

    def add_action_client(self, msg_type, topic):
        # TODO implement me!
        pass

    def add_import(self, module, include=None):
        if include:
            import_string = 'from {0} import {1}'.format(module, include)
        else:
            import_string = 'import {}'.format(module)
        if import_string not in self._imports:
            self._imports.append(import_string)

    def add_function(self, name, args, code):
        req_args = []
        opt_args = []
        if name == 'run':
            self._has_run = True
        sorted(args, key=itemgetter('name'))
        for arg in args:
            arg_name = arg['name']
            # if arg has default value it is optional, otherwise it is required
            if 'default' in arg:
                default = arg['default']
                # if type of default is str add necessary quotes, otherwise not
                if type(default) is str:
                    opt_args.append("{0}='{1}'".format(arg_name, default))
                else:
                    opt_args.append("{0}={1}".format(arg_name, default))
            else:
                req_args.append('{0}'.format(arg_name))
        arguments = 'self'
        if len(req_args):
            arguments += ', ' + ', '.join(req_args)
        if len(opt_args):
            arguments += ', ' + ', '.join(opt_args)
        function = ['def {0}({1}):'.format(name, arguments)]
        function.extend(code.splitlines())
        self._functions.append(self.make_intended_block(function, depth=2, underintend_first_line=True))

    def add_init_block(self, block):
        if type(block) != list:
            raise TypeError('block must be list of strings')
        self._add_init.extend(block)

    def set_intro(self,
                  author='Your Name',
                  copyright_='Copyright Date, Place',
                  credits_="['Pay', 'Your', 'Dues! ;-)']",
                  license_='LGPL v3',
                  version='3.14.159',
                  maintainer='Maintainer',
                  email='your.name@email.com',
                  status='freshly generated'):
        self._intro = list()
        self._intro.append('""" Auto-generated library, NEVER edit this! """')
        self._intro.append("__author__ = '{}'".format(author))
        self._intro.append("__copyright__ = '{}'".format(copyright_))
        self._intro.append("__credits__ = {}".format(credits_))
        self._intro.append("__license__ = '{}'".format(license_))
        self._intro.append("__version__ = '{}'".format(version))
        self._intro.append("__maintainer__ = '{}'".format(maintainer))
        self._intro.append("__email__ = '{}'".format(email))
        self._intro.append("__status__ = '{}'".format(status))

    @staticmethod
    def check_topic(topic):
        return '/' + topic if topic[0] is not '/' else topic

    def generate(self):
        blocks = list()

        # gen intro
        if not len(self._intro):
            self.set_intro()
        blocks.append('\n'.join(self._intro))

        # gen imports
        blocks.append(self.make_intended_block(self._imports, depth=0))

        # gen class
        blocks.append('\n\nclass {0}({1}):'.format(self._name, self._bases))

        if len(self._static_class_initials):
            blocks.append(self.make_intended_block(self._static_class_initials))

        # gen class __init__
        class_init = list()
        class_init.append('def __init__(self):')
        # gen class initials
        if len(self._class_initials):
            class_init.extend(self._class_initials)
        if len(self._param_init):
            class_init.append('# parameters')
            class_init.extend(self._param_init)
        if len(self._fsm_init):
            class_init.append('# finite state machine')
            class_init.append(self._fsm_init)
            class_init.extend(self._fsm_states)
            class_init.append(self._fsm_start_state)
        if len(self._pub_init):
            class_init.append('# publishers')
            class_init.extend(self._pub_init)
        if len(self._sub_init):
            class_init.append('# subscribers')
            class_init.extend(self._sub_init)
        if len(self._ss_init):
            class_init.append('# service servers')
            class_init.extend(self._ss_init)
        if len(self._sc_init):
            class_init.append('# service clients')
            class_init.extend(self._sc_init)
        if len(self._as_init):
            class_init.append('# action servers')
            class_init.extend(self._as_init)
        if len(self._ac_init):
            class_init.append('# action clients')
            class_init.extend(self._ac_init)
        if len(self._tfb_init):
            class_init.append('# tf broadcasters')
            class_init.extend(self._tfb_init)
        if len(self._tfl_init):
            class_init.append('# tf listeners')
            class_init.extend(self._tfl_init)

        # gen additional init blocks
        if len(self._add_init):
            class_init.append('# additional')
            class_init.extend(self._add_init)

        if len(class_init) is 1:
            class_init.append('# TODO: implement me!')
            class_init.append('pass')

        blocks.append(self.make_intended_block(class_init, depth=2, underintend_first_line=True))

        # gen class functions
        self._functions.append(self.make_intended_block(self._run_function, depth=2, underintend_first_line=True))
        blocks.extend(self._functions)

        # return library source code
        return '\n'.join(blocks)

    @staticmethod
    def generate_node(name, package, library):
        node = [
            "#!/usr/bin/env python",
            "import rospy",
            "",
            "from {0}.{1} import {1}".format(package, library),
            "",
            "if __name__ == '__main__':",
            "    rospy.init_node('{0}_node', anonymous=False)".format(name.lower()),
            "    node = {0}()".format(library),
            "    node.run()"
        ]
        return '\n'.join(node)

    @staticmethod
    def make_intended_block(block, depth=1, underintend_first_line=False):
        if type(block) is not list:
            raise TypeError('"block" must be list of strings, instead it is: {0}'.format(type(block)))
        intended_block = '\n'
        if underintend_first_line:
            intended_block += ((depth - 1) * 4) * ' '
        else:
            intended_block += (depth * 4) * ' '
        intended_block += ('\n' + (depth * 4) * ' ').join(block)
        return intended_block
