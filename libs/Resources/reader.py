class Reader:
    # reader class for getting data from the robot and passing to ROS
    def __init__(self, p):
        # constructor for settings class
        if type(p) != tuple and type(p) != list and type(p) != str:
            raise TypeError(
                "probes should be an itterable list/tupple or a string not %s" % type(p)
            )
        self.pub = None
        self.probes = p
        self.queue = {}
        for label in p:
            self.queue[label] = 0

    def publish(self, data):
        ## Publish data
        self.pub.publish(data)

    def add(self, name, val):
        # add data piece
        try:
            if type(self.probes) == str or name == self.probes:
                self.queue[name] = val
            if name in self.probes:
                self.queue[name] = val
        except Exception as e:
            raise e

    def needs(self, name):
        # check if name is used by this class
        return name == self.probes if type(self.probes) == str else name in self.probes

    def get_probes(self):
        # return an array of all probes
        return (
            self.probes
            if len(self.probes) > 1 or type(self.probes) == str
            else self.probes[0]
        )

    def get_probe_list(self):
        return self.probes if type(self.probes) != str else [self.probes]
