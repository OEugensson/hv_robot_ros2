import socket
import select
import struct
import code
import queue as Queue
import threading
import time
import os
import sys
import subprocess

# local imports
from libs.Resources.listener import *
from libs.Resources.utils import *
from libs.Resources.config import *

### --------- ###
#
### Harvest Vehicle Client Connection ###
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


class Client:
    def __init__(self, host):
        self.capture_buffer = ()
        self.probe_names = {}
        self.probe_defs = {}
        self.probe_list = []
        self.capturing = False
        self.listener_pipe = None
        self.listener = None
        self.found_probes = False
        self.listener_closed = True

        self.connect(host)
        # wait for the connection to be fully made
        time.sleep(3)

    def connect(self, host, port=4400):
        # Makes a connection to the robot.
        # performs an initialization handshake to get version, tick rate and probes
        # starts the listener subclass instance to handle recieving msgs

        try:
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
            while len(self.probe_defs.keys()) < TOTAL_PROBES:
                self.discover_probes()
                time.sleep(0.1)
            # toggle state to record that the probes have been looked up and stored
            self.found_probes = True

            print( "Protocol version = 0x%04x, sample rate is %d Hz, %d probes" % (
                self.version,
                self.hz,
                len(self.probe_defs.keys()),
            ))
        except Exception as e:
            print ("Client Connection Failed")
            raise e

    def disconnect(self):
        try:
            # end the connection
            self.stop_listener()
            while not self.listener_closed:
                time.sleep(0.5)
            self.s.close()
            print ("Client has disconnected")
        except Exception as e:
            print ("Client Disconnet Failed")
            raise e

    def reconnect(self, host, port=4400):
        # attempt to reconnect the client to the server
        try:
            # clear up previous listener/socket connection
            # self.listener.join()
            self.s.close()

            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((host, port))
            self.recvbuf = ""
            print ("Reconnected")

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

            print ("Protocol version = 0x%04x, sample rate is %d Hz, %d probes" % (
                self.version,
                self.hz,
                len(self.probe_defs.keys()),
            ))
        except Exception as e:
            print ("Client Connection Failed")
            raise e

    ## ----- Probe Methods ----- ##

    def discover_probes(self):
        # gets all the available probes
        # probes are used to either get information or set values in the robot
        try:
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
        except Exception as e:
            print ("probe discovery failed")
            raise e
        finally:
            return

    def return_probes(self):
        # return the list of probes
        probe_code_list = []
        try:
            for id in self.probe_defs.keys():
                (name, type, length) = self.probe_defs[id]
                type_name = type_table[type][0]
                probe_code_list.append(name)
        except Exception as e:
            raise e
        finally:
            return probe_code_list

    def lookup_probe_id(self, probe):
        # get the information about a probe by passing either the integer code for the probe
        # or the string name of the probe
        try:
            if type(probe) == str:
                if not probe in self.probe_names:
                    raise ValueError("unknown probe %s" % probe)
                probe_id = self.probe_names[probe]
            elif type(probe) == int:
                probe_id = probe
            else:
                raise ValueError(
                    "probe must be int or string, probe is %s" % type(probe)
                )

            return probe_id
        except Exception as e:
            raise e

    def lookup_probe_type(self, probe_id):
        # get the probe type
        try:
            if type(probe_id) != int:
                raise ValueError("probe id must be an int")
            return self.probe_defs[probe_id][1]
        except Exception as e:
            raise e

    def lookup_probe(self, t):
        # get the probe info
        try:
            if not self.found_probes:
                raise RuntimeWarning("Probe list has not been discovered yet")
            if type(t) != int:
                raise ValueError("probe id must be an int")
            # print(t in self.probe_defs.keys())
            if t not in self.probe_defs.keys():
                raise LookupError("Probe %i not found in probe defs" % t)
            return (self.probe_defs[t][0], self.probe_defs[t][1])
        except Exception:
            raise

    def enable_probes(self, probes):
        # enables one or more probes.
        # allow for singleton argument
        try:
            if type(probes) != tuple:
                probes = ((probes),)
            v = ""
            for probe in probes:
                probe_id = self.lookup_probe_id(probe)
                v += struct.pack("<H", probe_id)
            self.send_message(make_tlv(MP_TLV_ENABLE_PROBES, v))
        except Exception as e:
            raise e

    def encode_probe_value(self, probe_type, value):
        # encode as tlv msg
        if type_table[probe_type][0] == "tlv":
            return value  # assume it's already encoded.
        elif type_table[probe_type][0] == "string":
            return value + "\0"  # add a null terminator
        else:
            return struct.pack(type_table[probe_type][1], value)

    def decode_probe_data(self, probe_type, data):
        # decode tlv msg
        if type_table[probe_type][0] == "tlv":
            return tlv_to_text(data)
        elif type_table[probe_type][0] == "string":
            #        return len(data)
            return data[:-1]  # lose the null terminator
        else:
            # try to unpack data
            # NOTE: this was added to prevent total falure when packets type data failed (specifically the float type)
            try:
                data = struct.unpack(type_table[probe_type][1], data)[0]
                return data
            except:
                raise RuntimeError("struct unpack failed")

    def start_probes(self):
        # start capturing information from a probe
        self.capturing = True
        self.send_message(make_tlv(MP_TLV_START_PROBES))

    def stop_probes(self):
        # stop capturing information from a probe
        self.send_message(make_tlv(MP_TLV_STOP_PROBES))
        self.capturing = False

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

    ## ---- Message Handler Methods ---- ##

    def send_message(self, message):
        # Call to send a msg

        # print "sending: ",
        # print struct.unpack("%dB" % len(message), message)
        self.s.send(message)

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

    def get_debug_message(self, message):
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
        return "Debug message (t = %d): %s" % (tick, message_text)

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

    ## ----- Data Capture Methods Not currently used in bridging function ----- ##

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

    ## ----- Listener Methods ----- ##

    def start_listener(self, n=10):
        # start listener conenction
        self.listener_pipe = os.pipe()

        self.listener_q = Queue.Queue(maxsize=n)

        self.listener = listen(self.listener_q, self)
        self.listener.start()
        self.listener_closed = False

    def stop_listener(self):
        # close listener connection
        # TODO: why does this line fail?
        try:
            os.write(self.listener_pipe[1], "bye")
            os.close(self.listener_pipe[1])
        except Exception as e:
            print ("listener pipe not closing")
            pass
        finally:
            self.listener.join()
            self.listener_closed = True

    ## ----- Script Methods ----- ##

    def send_script(self, scriptname):
        # Assume that script-compile is in the same directory as this script.
        compiler = sys.path[0] + "/script-compile"

        compiled_script = subprocess.Popen(
            [compiler, "--stdout", scriptname], stdout=subprocess.PIPE
        ).communicate()[0]
        self.write_probe("rcp_script", compiled_script)
