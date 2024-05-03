class Writer:
    # reader class for getting data from the robot and passing to ROS
    msgs = []
    probes = []

    def __init__(self, p):
        # constructor for settings class
        print("test4")
        if type(p) != tuple and type(p) != list and type(p) != str:
            raise TypeError(
                "probes should be an itterable list/tupple or a string not %s" % type(p)
            )
        print("test3")
        self.probes = p

    def get_msgs(self):
        # collect the messages to send
        m = self.msgs
        self.msgs = []
        return m

    def clear_msgs(self):
        # clear msg array
        self.msgs = []

    def callback(self, msg):
        # subscriber callback function
        if type(self.probes) == str:
            # if the message is a single item value pair
            self.msgs = [(self.probes, msg)]
        elif type(msg) == list:
            # handle an array of messages
            # each message coresponds to a probe (1 to 1)
            if len(msg) != len(self.probes):
                raise Exception("message size missmatch")
            self.msgs = zip(self.probes, msg)
        else:
            # publish the same message to each of the probes
            self.msgs = [(c, msg) for c in self.probes]
