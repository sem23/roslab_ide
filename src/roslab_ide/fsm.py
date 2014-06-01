__author__ = 'privat'


class FiniteStateMachine:

    def __init__(self):
        self.handlers = {}
        self.start_state = None
        self.end_states = []

    def add_state(self, name, handler=None):
        """
        Add new state to machine.

        If handler is None state will be set to an end_state
        :param name:
        :param handler:
        """
        name = name.upper()
        if handler:
            self.handlers[name] = handler
        else:
            self.end_states.append(name)

    def set_start(self, name):
        """
        Set machine start state.

        :type name: str
        :param name: Name of initial state.
        """
        self.start_state = name.upper()

    def run(self, cargo):
        """
        Run finite state machine.
        This method calls state handlers and enables transitions. It is the core of the machine.

        :param cargo: Initial payload(function arguments) for start state.
        :raise RuntimeError: If there are no states or the start state is missing a RuntimeError will be raised.
        """
        try:
            handler = self.handlers[self.start_state]
        except:
            raise RuntimeError('Must call .set_start() before .run() and start_state must have a handler!')
        if not len(self.end_states):
            raise RuntimeError('at least one state must be an end_state')

        while True:
            new_state, cargo = handler(cargo)
            if new_state.upper() in self.end_states:
                print("reached ", new_state)
                break
            else:
                handler = self.handlers[new_state.upper()]