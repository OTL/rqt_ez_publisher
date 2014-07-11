import copy
import re
import rospy
import roslib.message
import roslib.msgs
from functools import reduce

from rqt_py_common.topic_helpers import get_field_type


def get_field_type_capable_with_index(field_string):
    m = re.search('(.+)(\[[0-9]+\])', field_string)
    if m:
        return get_field_type(m.group(1))
    else:
        return get_field_type(field_string)


def make_topic_strings(msg_instance, string=''):
    if msg_instance is None:
        return string
    if isinstance(msg_instance, list):
        msg_type = get_field_type_capable_with_index(string)[0]
        if msg_type is not None:
            array_instance = msg_type()
            return make_topic_strings(array_instance, string + '[0]')
        else:
            print 'not found type of %s' % string
            return ''
    try:
        return [make_topic_strings(msg_instance.__getattribute__(slot),
                                   string + '/' + slot)
                for slot in msg_instance.__slots__]
    except AttributeError:
        return string


def set_msg_attribute_value(msg_instance, topic_name, type, attributes,
                            array_index, value):
    message_target = msg_instance
    if len(attributes) >= 2:
        full_string = topic_name
        for attr in attributes[:-1]:
            full_string += '/' + attr
            m = re.search('(\w+)\[([0-9]+)\]', attr)
            if m:
                index = int(m.group(2))
                attr = m.group(1)
                array_type = get_field_type_capable_with_index(full_string)[0]
                while len(message_target.__getattribute__(attr)) <= index:
                    message_target.__getattribute__(attr).append(array_type())
                message_target = message_target.__getattribute__(attr)[index]
            elif get_field_type_capable_with_index(full_string)[1]:  # this is impossible
                array_type = get_field_type_capable_with_index(full_string)[0]
                if len(message_target.__getattribute__(attr)) == 0:
                    message_target.__getattribute__(attr).append(array_type())
                message_target = message_target.__getattribute__(attr)[0]
            else:
                message_target = message_target.__getattribute__(attr)
    if array_index is not None:
        array = message_target.__getattribute__(attributes[-1])
        while len(array) <= array_index:
            array.append(type())
        array[array_index] = value
        message_target.__setattr__(attributes[-1], array)
    else:
        message_target.__setattr__(attributes[-1], value)
    message_target = value


def flatten(complicated_list):
    if isinstance(complicated_list, list):
        return reduce(lambda a, b: a + flatten(b), complicated_list, [])
    else:
        return [complicated_list]


def find_topic_name(full_text, topic_dict):
    if full_text == '':
        return (None, None, None)
    if full_text[0] != '/':
        full_text = '/' + full_text
    # This is topic
    if full_text in topic_dict:
        return (full_text, None, None)
    splited_text = full_text.split('/')[1:]
    topic_name = ''
    while topic_name not in topic_dict and splited_text:
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
    # for Header -> std_msgs/Header
    topic_type_str = roslib.msgs.resolve_type(topic_type_str, '')
    if not attributes:
        return (None, False)
    try:
        _, spec = roslib.msgs.load_by_type(topic_type_str)
    except roslib.msgs.MsgSpecException as e:
        return (None, False)
    try:
        head_attribute = attributes[0].split('[')[0]
        index = spec.names.index(head_attribute)
        field = spec.parsed_fields()[index]
        attr_type = field.base_type
        if field.is_builtin:
            if attr_type in ['int8', 'int16', 'int32', 'int64']:
                return (int, field.is_array)
            if attr_type in ['byte', 'uint8', 'uint16', 'uint32', 'uint64']:
                return ('uint', field.is_array)
            elif attr_type in ['float32', 'float64']:
                return (float, field.is_array)
            elif attr_type == 'string':
                return (str, field.is_array)
            elif attr_type == 'bool':
                return (bool, field.is_array)
            else:
                print 'not support %s' % attr_type
                return (None, False)
        else:
            return get_value_type(field.base_type, attributes[1:])
    except ValueError as e:
        return (None, False)
    return (None, False)


def make_text(topic_name, attributes, array_index):
    text = topic_name + '/' + '/'.join(attributes)
    if array_index is not None:
        text += '[%d]' % array_index
    return text


class TopicPublisher(object):

    def __init__(self, topic_name, message_class):
        self._name = topic_name
        self._publisher = rospy.Publisher(
            topic_name, message_class, queue_size=100)
        self._message = message_class()

    def get_topic_name(self):
        return self._name

    def publish(self):
        self._publisher.publish(self._message)

    def get_message(self):
        return self._message


class EasyPublisherModel(object):

    def __init__(self, publisher_class=TopicPublisher):
        self._publishers = {}
        self._publisher_class = publisher_class

    def publish_topic(self, topic_name):
        if topic_name in self._publishers:
            self._publishers[topic_name].publish()

    def get_publisher(self, topic_name):
        if topic_name in self._publishers:
            return self._publishers[topic_name]
        else:
            return None

    def _add_publisher_if_not_exists(self, topic_name, message_class):
        if topic_name not in self._publishers:
            self._publishers[topic_name] = self._publisher_class(
                topic_name, message_class)

    def get_topic_names(self):
        _, _, topic_types = rospy.get_master().getTopicTypes()
        return sorted([x[0] for x in topic_types])

    def expand_attribute(self, input_text, array_index=None):
        text = copy.copy(input_text)
        try:
            msg_type, is_array = get_field_type_capable_with_index(text)
            if is_array:
                if array_index is None:
                    text += '[0]'
                else:
                    array_string = '[%d]' % array_index
                    if not text.endswith(array_string):
                        text += array_string
            if msg_type == int: # for time ? not support
                return []
            elif msg_type:
                return flatten(make_topic_strings(msg_type(), text))
            else:
                return []
        except AttributeError as e:
            return []

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
        builtin_type, is_array = get_value_type(topic_type_str, attributes)
        return (topic_name, attributes, builtin_type, is_array, array_index)

    def shutdown(self):
        for pub in list(self._publishers.values()):
            pub.unregister()
