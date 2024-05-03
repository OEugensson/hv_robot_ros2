import socket
import select
import struct
import code
import threading
import queue as Queue
import time
import os
import sys
import subprocess

### --------- ###
#
### Mindprobe ###
#
# This is an adaptation of the mindprobe scripts as a class
# this adaptation was done to be able to better handle data
# and operate in a persistent state as part of the ros structure
#
# Due to the synchonus connection stream, data is transmitted in
# a single pipe, utilizing a tcp socket connection. because of this
# there can only be one Mindprobe connection made to the device.
#
# Note: there may be a potential to expand this to have multiple
# connections by using multiple ports. however I do not know which ports support
# a MP connection. currently only 4400 is known to be able to transmit data.
#
### -------- ###


###
# Type names for probe types (t in tlv)
###
MP_TLV_NULL = 0
MP_TLV_CONNECTED = 1
MP_TLV_DISCONNECTED = 2
MP_TLV_PROTOCOL_VERSION = 3
MP_TLV_DISCOVER_PROBES = 4
MP_TLV_HZ = 5
MP_TLV_PROBE_DEF = 6
MP_TLV_ENABLE_PROBES = 7
MP_TLV_START_PROBES = 8
MP_TLV_STOP_PROBES = 9
MP_TLV_PROBE_DATA = 10
MP_TLV_CURRENT_TICK = 11
MP_TLV_MISSED_DATA = 12
MP_TLV_DEBUG_MESSAGE = 13
MP_TLV_MESSAGE_TEXT = 14
MP_TLV_MISSED_DEBUG_MESSAGES = 15
MP_TLV_WRITE_PROBES = 16

###
# functions that handle the formatting, reading and witing of tlv messages
# tlv is the structure of message that mp uses to communicate
# t = type
# l = length
# v = value
###
def make_tlv_uint8(t, v):
    return struct.pack("<HHH", t, 1, v)


def make_tlv_uint16(t, v):
    return struct.pack("<HHH", t, 2, v)


def make_tlv_uint32(t, v):
    return struct.pack("<HHI", t, 4, v)


def make_tlv(t, v=""):
    return struct.pack("<HH", t, len(v)) + v


def tlv_len(message):
    (t, l) = struct.unpack("<HH", message[0:4])
    return l


def next_tlv(message):
    (t, l) = struct.unpack("<HH", message[0:4])
    v = message[4 : 4 + l]
    return (t, l, v, message[4 + l :])


def tlv_to_text(data):
    (t, l) = struct.unpack("<HH", data[0:4])
    v = data[4 : 4 + l]
    # If we knew which t values were recursive encodings, we could
    # be clever here.
    string = "t=%d:l=%d:v=" % (t, l)
    for i in range(len(v)):
        string += "%02x" % ord(v[i])
        if i < len(v):
            string += ":"
    return string


class Mindprobe:
    # Mindprobe class
    # each mindproble class represents a connection to a HAI bot
    # there can only be one connection, and therefore one mp class per port
    # the default port is 4400. (other working ports are unknown)
    def __init__(self):
        # constructor for the mindprobe instatnce
        self.capture_buffer = ()
        self.probe_names = {}
        self.probe_defs = {}
        self.type_table = {
            1: ("uint8", "<B"),  # 1-byte unsigned integer
            2: ("int8", "<b"),  # 1-byte signed integer
            3: ("uint16", "<H"),  # 2-byte unsigned integer
            4: ("int16", "<h"),  # 2-byte signed integer
            5: ("uint32", "<I"),  # 4-byte unsigned integer
            6: ("int32", "<i"),  # 4-byte signed integer
            7: ("uint64", "<Q"),  # 8-byte unsigned integer
            8: ("int64", "<q"),  # 8-byte signed integer
            9: ("float", "<f"),  # 4-byte float (IEEE 754)
            10: ("double", "<d"),  # 8-byte float (IEEE 754)
            11: ("tlv", ""),  # Variable-length TLV data
            12: ("bool", "<B"),  # 1-byte boolean integer (1 or 0)
            13: ("string", ""),  # variable-size null-terminated char string
        }
        self.probe_list = []
        self.listener_pipe = None

    def init(self, host):
        # initialize the mp connection
        self.hostname = host
        self.connect(host)  # sets up the connection

    def start(self):
        # NOTE: is fake start required
        # self.enable_probes(1125) # enables the virtual start probe
        FAKE_START = 1125
        STANDBY_MODE = 2877
        ARM_ENABLED = 1082
        GRIPPER_ENABLED = 1083
        DRIVE_ENABLED = 580
        DRIVE_COMMAND = 1220
        ROB_SENSOR = 3100
        LINE_SENSOR = 3171
        I2C_SENSOR = 3895

        BEEP = 1120

        self.write_probe(BEEP, 0)
        self.write_probe(FAKE_START, 1)
        self.write_probe(STANDBY_MODE, 0)
        # self.write_probes([(STANDBY_MODE,1),(DRIVE_ENABLED,1),(DRIVE_COMMAND,1),(ROB_SENSOR,1),(LINE_SENSOR,1),(I2C_SENSOR,1),(ARM_ENABLED,1),(GRIPPER_ENABLED,1)]) # sets the start to TRUE. this 'starts' the robot.

        time.sleep(2)
        self.init_probes()  # enable all the queued probes

    def stop(self):
        # end everything that is running
        # sets the start to FALSE. this 'stops' the robot.
        # sets flag pulled to true
        self.write_probes([(2426, 1), (1125, 0), (2419, 1)])
        print ("virtual e-stop triggered")
        time.sleep(1)
        self.write_probes([(2426, 0), (2419, 0)])
        print (str(self.hostname) + " has stopped all running processes")

    def connect(self, host, port=4400):
        # Makes a connection to the robot.
        # performs an initialization handshake to get version, tick rate and probes
        # starts the listener subclass instance to handle recieving msgs

        self.capture_buffer_lock = threading.Semaphore()

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((host, port))
        self.recvbuf = ""
        print ("Connected")

        self.start_listener()

        # Expect the server to send us the protocol version, unsolicited:
        (t, l, v) = self.get_next_message()
        if (t != MP_TLV_PROTOCOL_VERSION) or (l != 2):
            raise RuntimeError("unexpected message t=%d l=%d" % (t, l))
        self.version = struct.unpack("<H", v)[0]

        # Query the tick rate:
        self.send_message(make_tlv(MP_TLV_HZ))
        (t, l, v) = self.get_next_message()
        while t != MP_TLV_HZ:
            print ("message id {}").format(t)
            time.sleep(0.1)
            (t, l, v) = self.get_next_message()
        if (t != MP_TLV_HZ) or (l != 2):
            raise RuntimeError("unexpected message t=%d l=%d" % (t, l))
        self.hz = struct.unpack("<H", v)[0]

        # Discover the probes:
        time.sleep(0.5)
        while len(self.probe_defs.keys()) < 3900:
            self.discover_probes()
            time.sleep(0.1)

        print ("Protocol version = 0x%04x, sample rate is %d Hz, %d probes" % (
            self.version,
            self.hz,
            len(self.probe_defs.keys()),
        ))

    def disconnect(self):
        try:
            # call e-stop to and all functionality before breaking connection
            self.stop()

            # end the connection
            self.stop_listener()
            self.s.close()
            print ("mindprobe has disconnected")
            return True
        except Exception as e:
            print ("mp disconnect has failed:")
            print (e)
            return False

    def discover_probes(self):
        # gets all the available probes
        # probes are used to either get information or set values in the robot

        self.send_message(make_tlv(MP_TLV_DISCOVER_PROBES))
        while True:
            (t, l, def_items) = self.get_next_message()
            if t != MP_TLV_DISCOVER_PROBES:
                raise RuntimeError("unexpected message t=%d l=%d" % (t, l))

            while len(def_items):
                (t, l, v, def_items) = next_tlv(def_items)
                if t != MP_TLV_PROBE_DEF:
                    raise RuntimeError("unexpected message t=%d l=%d" % (t, l))

                (probe_id, probe_type, probe_length) = struct.unpack("<HHH", v[0:6])

                # Name has null terminator, which Python doesn't need.
                probe_name = v[6 : len(v) - 1]

                self.probe_names[probe_name] = probe_id
                self.probe_defs[probe_id] = (probe_name, probe_type, probe_length)
                pass

            if len(def_items) == 0 or self.version < 0x0104:
                break

            pass

        return

    def show_probes(self):
        # print out the probes
        for id in self.probe_defs.keys():
            (name, type, length) = self.probe_defs[id]
            type_name = self.type_table[type][0]
            print ("probe id %d: %s type %s length %d" % (id, name, type_name, length))

    def return_probes(self):
        # return the list of probes
        probe_code_list = []
        for id in self.probe_defs.keys():
            (name, type, length) = self.probe_defs[id]
            type_name = self.type_table[type][0]
            probe_code_list.append(name)
        return probe_code_list

    def send_message(self, message):
        # Call to send a msg

        # print "sending: ",
        # print struct.unpack("%dB" % len(message), message)
        self.s.send(message)

    def lookup_probe_id(self, probe):
        # get the information about a probe by passing either the integer code for the probe
        # or the string name of the probe
        if type(probe) == str:
            if not probe in self.probe_names:
                raise ValueError("unknown probe %s" % probe)
            probe_id = self.probe_names[probe]
        elif type(probe) == int:
            probe_id = probe
        else:
            raise ValueError("probe must be int or string")

        return probe_id

    def enable_probes(self, probes):
        # enables one or more probes.

        # allow for singleton argument
        if type(probes) != tuple:
            probes = ((probes),)
        v = ""
        for probe in probes:
            probe_id = self.lookup_probe_id(probe)
            v += struct.pack("<H", probe_id)
        self.send_message(make_tlv(MP_TLV_ENABLE_PROBES, v))

    def include_probes(self, probes):
        # adds a probe code or collection of probe codes to the collection of probes
        if type(probes) != tuple:
            probes = ((probes),)
        for probe in probes:
            if probe not in self.probe_list:
                self.probe_list.append(probe)

    def init_probes(self):
        # enables all the probes on the start list
        self.enable_probes(tuple(self.probe_list))
        # self.probe_list = []

    def start_probes(self):
        # start capturing information from a probe
        self.capturing = True
        self.send_message(make_tlv(MP_TLV_START_PROBES))

    def stop_probes(self):
        # stop capturing information from a probe
        self.send_message(make_tlv(MP_TLV_STOP_PROBES))
        self.capturing = False

    def do_debug_message(self, message):
        # handles debug msgs
        (t, l, v, message) = next_tlv(message)
        if t != MP_TLV_CURRENT_TICK:
            raise RuntimeError("unexpected type = %d" % t)
        tick = struct.unpack("<I", v)[0]
        (t, l, v, message) = next_tlv(message)
        if t != MP_TLV_MESSAGE_TEXT:
            raise RuntimeError("unexpected type = %d" % t)
        message_text = v[0 : len(v) - 1]
        if message_text[-1] == "\n":
            message_text = message_text[0 : len(message_text) - 1]
        print ("Debug message (t = %d): %s" % (tick, message_text))

    def do_async_message(self, t, l, v):
        # async msg handler
        if t == MP_TLV_PROBE_DATA:
            self.capture_data(v)
            return True
        elif t == MP_TLV_DEBUG_MESSAGE:
            self.do_debug_message(v)
            return True
        elif t == MP_TLV_MISSED_DATA:
            print ("Missed %d probe messages" % struct.unpack("<I", v))
            return True
        elif t == MP_TLV_MISSED_DEBUG_MESSAGES:
            print ("Missed %d debug messages" % struct.unpack("<I", v))
            return True
        else:
            return False  # not an asynchronous message

    def receive_next_message(self):
        # Receive one message from the socket:
        while (len(self.recvbuf) < 4) or (len(self.recvbuf) < tlv_len(self.recvbuf)):
            self.recvbuf += self.s.recv(65536)
            # print "recvbuf is: ",
            # print struct.unpack("%dB" % len(recvbuf), recvbuf)

        (t, l, v, self.recvbuf) = next_tlv(self.recvbuf)

        return (t, l, v)

    def get_next_message(self):
        # gets the next msg from the queue
        (t, l, v) = self.listener_q.get()
        return (t, l, v)

    def capture_buffer_len(self):
        # length of capture buffer
        self.capture_buffer_lock.acquire()
        l = len(self.capture_buffer)
        self.capture_buffer_lock.release()
        return l

    def capture(self, t):
        # get data from the bot over an interval t
        # add it to the capture_buffer
        n = int(round(t * self.hz))

        self.capture_buffer_lock.acquire()
        self.capture_buffer = ()
        self.capture_buffer_lock.release()

        self.start_probes()
        time.sleep(t)  # let the listener thread do the capturing
        self.stop_probes()

    def capture_data(self, data):
        # Add the data an item in the caputre buffer.
        if self.capturing:
            self.capture_buffer_lock.acquire()
            self.capture_buffer += (data,)
            self.capture_buffer_lock.release()

    def return_capture(self, interval=None):
        # get data from the capture buffer and return it

        if interval == None:
            interval = 1.0 / self.hz

        step = int(round(self.hz * interval))

        captured_data_array = []

        self.capture_buffer_lock.acquire()
        for i in range(0, len(self.capture_buffer), step):
            item = self.capture_buffer[i]

            if len(item) == 0:
                print ("empty capture buffer item")
                continue

            (t, l, v, item) = next_tlv(item)
            if t != MP_TLV_CURRENT_TICK:
                print ("bad capture item, first element not tick")
                continue

            (val,) = struct.unpack("<I", v)
            # print 'tick =', val,

            captured_data_obj = {}
            while len(item):
                (t, l, v, item) = next_tlv(item)

                try:
                    name = self.probe_defs[t][0]
                    type = self.probe_defs[t][1]
                    val = self.decode_probe_data(type, v)

                    captured_data_obj[name] = val
                except RuntimeError:
                    print (
                        "name: ",
                        str(name),
                        " type: ",
                        str(type),
                        " val: ",
                        str(val),
                    )

                # print name, '=', val,
            # print ""
            captured_data_array.append(captured_data_obj)
        self.capture_buffer_lock.release()
        return captured_data_array

    def show_capture(self, interval=None):
        # get data from the capture buffer and print it to console

        if interval == None:
            interval = 1.0 / self.hz

        step = int(round(self.hz * interval))

        self.capture_buffer_lock.acquire()
        for i in range(0, len(self.capture_buffer), step):
            item = self.capture_buffer[i]

            if len(item) == 0:
                print ("empty capture buffer item")
                continue

            (t, l, v, item) = next_tlv(item)
            if t != MP_TLV_CURRENT_TICK:
                print ("bad capture item, first element not tick")
                continue

            (val,) = struct.unpack("<I", v)
            print ("tick =", val,)

            print ("Probes: ", len(self.probe_defs))
            while len(item):
                (t, l, v, item) = next_tlv(item)

                if self.probe_defs[t] == None:
                    pass

                name = self.probe_defs[t][0]
                type = self.probe_defs[t][1]
                val = self.decode_probe_data(type, v)

                print (name, "=", val)
            print ("")
        self.capture_buffer_lock.release()

    def write_probe(self, probe, value):
        # write a vale to a probe
        probe_id = self.lookup_probe_id(probe)
        probe_type = self.probe_defs[probe_id][1]

        v = make_tlv(probe_id, self.encode_probe_value(probe_type, value))
        self.send_message(make_tlv(MP_TLV_WRITE_PROBES, v))

    def write_probes(self, probevals):
        # write a set of values to a set of probes
        v = ""

        for (probe, value) in probevals:
            probe_id = self.lookup_probe_id(probe)
            probe_type = self.probe_defs[probe_id][1]

            v += make_tlv(probe_id, self.encode_probe_value(probe_type, value))
        self.send_message(make_tlv(MP_TLV_WRITE_PROBES, v))

    def encode_probe_value(self, probe_type, value):
        # encode as tlv msg
        if self.type_table[probe_type][0] == "tlv":
            return value  # assume it's already encoded.
        elif self.type_table[probe_type][0] == "string":
            return value + "\0"  # add a null terminator
        else:
            return struct.pack(self.type_table[probe_type][1], value)

    def decode_probe_data(self, probe_type, data):
        # decode tlv msg
        if self.type_table[probe_type][0] == "tlv":
            return tlv_to_text(data)
        elif self.type_table[probe_type][0] == "string":
            #        return len(data)
            return data[:-1]  # lose the null terminator
        else:
            # try to unpack data
            # NOTE: this was added to prevent total falure when packets type data failed (specifically the float type)
            try:
                data = struct.unpack(self.type_table[probe_type][1], data)[0]
                return data
            except:
                raise RuntimeError("struct unpack failed")

    def send_script(self, scriptname):
        # Assume that script-compile is in the same directory as this script.
        compiler = sys.path[0] + "/script-compile"

        compiled_script = subprocess.Popen(
            [compiler, "--stdout", scriptname], stdout=subprocess.PIPE
        ).communicate()[0]
        self.write_probe("rcp_script", compiled_script)

    class listen(threading.Thread):
        # listener subclass for getting data from robot
        def __init__(self, q, mp):
            threading.Thread.__init__(self)
            self.q = q
            self.mp = mp
            self.setDaemon(True)

        def run(self):
            # runs a listener thread, reading in data and placing it in a queue
            while True:
                # Wait for incoming data from the RCP or for the listener
                # pipe to close, indicating that we are
                (r, w, x) = select.select((self.mp.s, self.mp.listener_pipe[0]), (), ())
                if self.mp.listener_pipe[0] in r:
                    # listener pipe signaled the thread to exit
                    os.close(self.mp.listener_pipe[0])
                    return
                if self.mp.s in r:
                    (t, l, v) = self.mp.receive_next_message()
                    if not self.mp.do_async_message(t, l, v):
                        self.q.put((t, l, v))
                    # time.sleep(0.1)

    def start_listener(self):
        # start listener conenction
        self.listener_pipe = os.pipe()

        self.listener_q = Queue.Queue()

        self.listener = self.listen(self.listener_q, self)
        self.listener.start()

    def stop_listener(self):
        # close listener connection
        # TODO: why does this line fail?
        # os.write(self.listener_pipe[1], 'bye')
        os.close(self.listener_pipe[1])
        self.listener.join()
