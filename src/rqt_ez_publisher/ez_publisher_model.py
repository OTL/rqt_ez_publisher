import re
import rospy
import roslib.message
import roslib.msgs
from functools import reduce

def make_topic_strings(t, string=''):
    try:
        return [make_topic_strings(t.__getattribute__(slot), string + '/' + slot) for slot in t.__slots__]
    except AttributeError as e:
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
    except roslib.msgs.MsgSpecException as e:
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
    except ValueError as e:
        return (None, False)
    return (None, False)


def make_text(topic_name, attributes, array_index):
    text = topic_name + '/' + '/'.join(attributes)
    if array_index != None:
        text += '[%d]' % array_index
    return text


class EasyPublisherModel(object):

    def __init__(self):
        self._publishers = {}
        self._messages = {}

    def get_publisher(self, topic_name):
        if topic_name in self._publishers:
            return self._publishers[topic_name]
        else:
            return None

    def get_message(self, topic_name):
        if topic_name in self._messages:
            return self._messages[topic_name]
        else:
            return None

    def _add_publisher_if_not_exists(self, topic_name, message_class):
        if not topic_name in self._publishers:
            self._publishers[topic_name] = rospy.Publisher(
                topic_name, message_class, queue_size=100)

    def _add_message_if_not_exists(self, topic_name, message_class):
        if not topic_name in self._messages:
            self._messages[topic_name] = message_class()

    def get_topic_break_down_strings(self, topic_name):
        return flatten(make_topic_strings(self._messages[topic_name],
                                          topic_name))

    def get_topic_names(self):
        _, _, topic_types = rospy.get_master().getTopicTypes()
        return sorted([x[0] for x in topic_types])

    def resister_topic_by_text(self, text):
        _, _, topic_types = rospy.get_master().getTopicTypes()
        topic_dict = dict(topic_types)
        topic_name, attributes, array_index = find_topic_name(text, topic_dict)
        if not topic_name:
            rospy.logerr('%s not found' % text)
            return None
        topic_type_str = topic_dict[topic_name]
        message_class = roslib.message.get_message_class(topic_type_str)
        self._add_publisher_if_not_exists(topic_name, message_class)
        self._add_message_if_not_exists(topic_name, message_class)
        builtin_type, is_array = get_value_type(topic_type_str, attributes)
        return (topic_name, attributes, builtin_type, is_array, array_index)

    def shutdown(self):
        for pub in list(self._publishers.values()):
            pub.unregister()
