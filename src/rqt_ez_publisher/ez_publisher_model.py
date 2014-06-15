import re
import rospy
import roslib.message
import roslib.msgs
import sys


def make_topic_strings(t, string=''):
    try:
        return [make_topic_strings(t.__getattribute__(slot), string + '/' + slot) for slot in t.__slots__]
    except AttributeError, e:
        return string


def flatten(L):
    if isinstance(L, list):
        return reduce(lambda a, b: a + flatten(b), L, [])
    else:
        return [L]


def find_topic_name(full_text, topic_dict):
    if full_text[0] != '/':
        full_text = '/' + full_text
    # This is topic
    if full_text in topic_dict:
        return (full_text, None, None)
    splited_text = full_text.split('/')[1:]
    topic_name = ''
    while not topic_name in topic_dict and splited_text:
        topic_name += '/' + splited_text[0]
        splited_text = splited_text[1:]
    if splited_text:
        m = re.search('(\w+)\[([0-9]+)\]', splited_text[-1])
        if m:
            splited_text[-1] = m.group(1)
            return (topic_name, splited_text, int(m.group(2)))
        else:
            return (topic_name, splited_text, None)
    else:
        return (None, None, None)


def get_value_type(topic_type_str, attributes):
    if not attributes:
        return (None, False)
    try:
        _, spec = roslib.msgs.load_by_type(topic_type_str)
    except roslib.msgs.MsgSpecException, e:
        return (None, False)
    try:
        index = spec.names.index(attributes[0])
        field = spec.parsed_fields()[index]
        attr_type = field.base_type
        if field.is_builtin:
            if attr_type in ['int8', 'int16', 'int32', 'int64']:
                return (int, field.is_array)
            if attr_type in ['uint8', 'uint16', 'uint32', 'uint64']:
                return ('uint', field.is_array)
            elif attr_type in ['float32', 'float64']:
                return (float, field.is_array)
            elif attr_type == 'string':
                return (str, field.is_array)
            elif attr_type == 'bool':
                return (bool, field.is_array)
        else:
            return get_value_type(field.base_type, attributes[1:])
    except ValueError, e:
        return (None, False)
    return (None, False)


def make_text(topic_name, attributes, array_index):
    text = topic_name + '/' + '/'.join(attributes)
    if array_index != None:
        text += '[%d]' % array_index
    return text
