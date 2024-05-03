import struct

### various utility functions shared by multiple modules ###


def twist_to_vel(twist):
    # differential drive conversion for a twist message
    # assumes x is forward/backward and y is lateral.
    r = 0.25
    L = 1.0
    v = twist.linear.x
    th = twist.angular.z
    left = (2 * v - L * th) / 2 * r
    right = (2 * v + L * th) / 2 * r
    return [left, right]


def resource_name(name):
    # formats a hostname to be legaly used in a ros pubisher/subscriber name
    return ((name.replace("-", "")).replace(".", "_")).lower()


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
