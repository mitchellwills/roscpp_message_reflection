#!/usr/bin/env python

import rospy
from roscpp_message_reflection.msg import *
from roscpp_message_reflection.srv import *
import re

builtins = ['bool', 'char', 'byte','int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32', 'float64', 'string', 'time', 'duration']

def get_service(type):
    (package, service_name) = type.split("/")
    package_srvs = __import__(package + ".srv").srv
    return getattr(package_srvs, service_name)

def get_message(type):
    if type in builtins:
        return None
    (package, message_name) = type.split("/")
    package_msgs = __import__(package + ".msg").msg
    return getattr(package_msgs, message_name)

def get_message_info(msg):
    if msg is None:
        return None
    info = MessageInfo(type=msg._type, md5sum=msg._md5sum, definition=msg._full_text)
    for i in xrange(len(msg.__slots__)):
        info.fields.append(MessageFieldInfo(name=msg.__slots__[i], type=msg._slot_types[i]))
    return info

def get_service_info(srv):
    if srv is None:
        return None
    msgs = {}
    msgs.update(get_message_info_rec(srv._request_class))
    msgs.update(get_message_info_rec(srv._response_class))
    return (ServiceInfo(type=srv._type, md5sum=srv._md5sum, req_type=srv._request_class._type, res_type=srv._response_class._type), msgs)

# prepend package name
def to_full_type(type, package_context):
    if type in builtins:
        return type
    if "/" in type:
        return type
    return package_context + "/" + type

FIELD_TYPE_REGEX = re.compile("([^\[]+)(?:\[\])?")
def get_message_info_rec(msg):
    info = get_message_info(msg)
    if info is None:
        return {}
    (package, message_name) = info.type.split("/")
    infos = {info.type: info}
    for field in info.fields:
        array_match = FIELD_TYPE_REGEX.match(field.type)
        value_type = array_match.group(1)
        infos.update(get_message_info_rec(get_message(to_full_type(value_type, package))))
    return infos

def fufill_message_info_request(req):
    return GetMessageInfoResponse(type=req.type, infos=get_message_info_rec(get_message(req.type)).values())

def fufill_service_info_request(req):
    (info, msgs) = get_service_info(get_service(req.type))
    return GetServiceInfoResponse(service=info, message_infos=msgs.values())

if __name__=="__main__":
    rospy.init_node("message_description_server")

    msg_service = rospy.Service("get_message_info", GetMessageInfo, fufill_message_info_request)
    srv_service = rospy.Service("get_service_info", GetServiceInfo, fufill_service_info_request)

    rospy.spin();
