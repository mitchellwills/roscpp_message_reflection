#!/usr/bin/env python

import rospy
from roscpp_message_reflection.msg import *
from roscpp_message_reflection.srv import *
import re

builtins = ['bool', 'char', 'byte','int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32', 'float64', 'string', 'time', 'duration']

def get_message(type):
    (package, message_name) = type.split("/")
    package_msgs = __import__(package + ".msg").msg
    return getattr(package_msgs, message_name)

def get_message_info(type):
    if type in builtins:
        return None
    msg = get_message(type)
    info = MessageInfo(type=msg._type, md5sum=msg._md5sum, definition=msg._full_text)
    for i in xrange(len(msg.__slots__)):
        info.fields.append(MessageFieldInfo(name=msg.__slots__[i], type=msg._slot_types[i]))
    return info

# prepend package name
def to_full_type(type, package_context):
    if type in builtins:
        return type
    if "/" in type:
        return type
    return package_context + "/" + type

FIELD_TYPE_REGEX = re.compile("([^\[]+)(?:\[\])?")
def get_message_info_rec(type):
    info = get_message_info(type)
    if info is None:
        return {}
    (package, message_name) = type.split("/")
    infos = {type: info}
    for field in info.fields:
        array_match = FIELD_TYPE_REGEX.match(field.type)
        value_type = array_match.group(1)
        infos.update(get_message_info_rec(to_full_type(value_type, package)))
    return infos

def fufill_message_info_request(req):
    return GetMessageInfoResponse(type=req.type, infos=get_message_info_rec(req.type).values())

if __name__=="__main__":
    rospy.init_node("message_info_node")
    service = rospy.Service("get_message_info", GetMessageInfo, fufill_message_info_request)

    rospy.spin();
