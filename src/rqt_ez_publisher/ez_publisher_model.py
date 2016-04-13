import copy
import functools
import re
import roslib.message
import roslib.msgs
import rospy
from rqt_py_common import topic_helpers as helpers


def get_field_type_capable_with_index(field_string):
    '''get type even if it contains [] for array'''

    m = re.search('(.+)(\[[0-9]+\])$', field_string)
    if m:
        return helpers.get_field_type(m.group(1))
    else:
        return helpers.get_field_type(field_string)


def make_topic_strings_internal(msg_instance, string='', modules=[]):
    '''returns break down strings'''

    if msg_instance is None:
        return [string]
    if isinstance(msg_instance, list):
        msg_type = get_field_type_capable_with_index(string)[0]
        if msg_type is not None:
            array_instance = msg_type()
            return make_topic_strings_internal(array_instance, string + '[0]', modules=modules)
        else:
            rospy.logwarn('not found type of %s' % string)
            return []
    # this should be replaced by plugin system
    for module in modules:
        if isinstance(msg_instance, module.get_msg_class()):
            return [string]
    try:
        return [make_topic_strings_internal(msg_instance.__getattribute__(slot),
                                            string + '/' + slot, modules=modules)
                for slot in msg_instance.__slots__]
    except AttributeError:
        return [string]


def set_msg_attribute_value(msg_instance, topic_name, msg_type, attributes,
                            array_index, value):
    '''set value to the attribute of topic'''
    message_target = msg_instance
    if len(attributes) >= 2:
        message_target = get_msg_attribute_value(
            message_target, topic_name, attributes[:-1])
    if array_index is not None:
        array = message_target.__getattribute__(attributes[-1])
        while len(array) <= array_index:
            array.append(msg_type())
        array[array_index] = value
        message_target.__setattr__(attributes[-1], array)
    else:
        message_target.__setattr__(attributes[-1], value)
    message_target = value


def get_msg_attribute_value(msg_instance, topic_name, attributes):
    message_target = msg_instance
    full_string = topic_name
    for attr in attributes:
        full_string += '/' + attr
        m = re.search('(\w+)\[([0-9]+)\]$', attr)
        if m:
            index = int(m.group(2))
            attr = m.group(1)
            array_type = get_field_type_capable_with_index(full_string)[0]
            while len(message_target.__getattribute__(attr)) <= index:
                message_target.__getattribute__(attr).append(array_type())
            message_target = message_target.__getattribute__(attr)[index]
        elif get_field_type_capable_with_index(full_string)[1]:
            array_type = get_field_type_capable_with_index(full_string)[0]
            if len(message_target.__getattribute__(attr)) == 0:
                message_target.__getattribute__(attr).append(array_type())
            message_target = message_target.__getattribute__(attr)[0]
        else:
            message_target = message_target.__getattribute__(attr)
    return message_target


def flatten(complicated_list):
    if isinstance(complicated_list, list):
        return functools.reduce(lambda a, b:
                                a + flatten(b), complicated_list, [])
    else:
        return [complicated_list]


def make_topic_strings(msg_instance, string='', modules=[]):
    return flatten(make_topic_strings_internal(msg_instance, string=string, modules=modules))


def find_topic_name(full_text, topic_dict):
    if full_text == '':
        return (None, None, None)
    if full_text[0] != '/':
        full_text = '/' + full_text
    # This is topic
    if full_text in topic_dict:
        return (full_text, None, None)
    splited_text = full_text.split('/')[1:]
    topic_name = full_text
    topic_inside_variable_list = []
    while topic_name not in topic_dict and splited_text:
        topic_name = topic_name.rstrip('/'.join(splited_text[-1]))
        topic_inside_variable_list.append(splited_text[-1])
        splited_text = splited_text[:-1]

    topic_inside_variable_list.reverse()
    if topic_name != '' and topic_inside_variable_list:
        m = re.search('(\w+)\[([0-9]+)\]', topic_inside_variable_list[-1])
        if m:
            topic_inside_variable_list[-1] = m.group(1)
            return (topic_name, topic_inside_variable_list, int(m.group(2)))
        else:
            return (topic_name, topic_inside_variable_list, None)
    else:
        return (None, None, None)


def get_value_type(topic_type_str, attributes, modules=[]):
    # for Header -> std_msgs/Header
    topic_type_str = roslib.msgs.resolve_type(topic_type_str, '')
    spec = None
    if not attributes:
        return (None, False)
    try:
        _, spec = roslib.msgs.load_by_type(topic_type_str)
    except roslib.msgs.MsgSpecException:
        return (None, False)
    except IOError as e:  # not found
        # for devel environment
        import os
        cmake_prefix_list = os.environ.get('CMAKE_PREFIX_PATH')
        if cmake_prefix_list is not None:
            package, msg = topic_type_str.split('/')
            for path in cmake_prefix_list.split(':'):
                msg_path = "%s/share/%s/msg/%s.msg" % (path, package, msg)
                if os.path.exists(msg_path):
                    _, spec = roslib.msgs.load_from_file(msg_path, package)
                    rospy.logdebug(
                        'loaded %s/%s for devel environment' % (package, msg))
                    break
    try:
        head_attribute = attributes[0].split('[')[0]
        index = spec.names.index(head_attribute)
        field = spec.parsed_fields()[index]
        attr_type = field.base_type
        if field.is_builtin:
            if attr_type in ['int8', 'int16', 'int32', 'int64']:
                return_type = int
            elif attr_type in ['byte', 'uint8', 'uint16', 'uint32', 'uint64']:
                return_type = 'uint'
            elif attr_type in ['float32', 'float64']:
                return_type = float
            elif attr_type == 'string':
                return_type = str
            elif attr_type == 'bool':
                return_type = bool
            else:
                rospy.logwarn('%s is not supported' % attr_type)
                return (None, False)
            return (return_type, field.is_array)
        else:
            for module in modules:
                if field.base_type == module.get_msg_string():
                    return (module.get_msg_string(), field.is_array)
            return get_value_type(field.base_type, attributes[1:], modules=modules)
    except ValueError as e:
        rospy.logwarn(e)
        return (None, False)
    return (None, False)


def make_text(topic_name, attributes, array_index):
    text = topic_name + '/' + '/'.join(attributes)
    if array_index is not None:
        text += '[%d]' % array_index
    return text


class EzPublisherModel(object):

    '''Model for rqt_ez_publisher'''

    def __init__(self, publisher_class, modules=[]):
        self._publishers = {}
        self._publisher_class = publisher_class
        self._modules = modules

    def get_modules(self):
        return self._modules

    def set_modules(self, modules):
        self._modules = modules

    def add_module(self, module):
        self._modules.append(module)

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
            if msg_type == int:  # for time ? not support
                return []
            elif msg_type:
                return make_topic_strings(msg_type(), text, modules=self._modules)
            else:
                return []
        except AttributeError:
            return []

    def register_topic_by_text(self, text):
        _, _, topic_types = rospy.get_master().getTopicTypes()
        topic_dict = dict(topic_types)
        topic_name, attributes, array_index = find_topic_name(text, topic_dict)
        if not topic_name:
            return None
        topic_type_str = topic_dict[topic_name]
        message_class = roslib.message.get_message_class(topic_type_str)
        self._add_publisher_if_not_exists(topic_name, message_class)
        builtin_type, is_array = get_value_type(
            topic_type_str, attributes, modules=self._modules)
        return (topic_name, attributes, builtin_type, is_array, array_index)

    def shutdown(self):
        for pub in list(self._publishers.values()):
            pub.unregister()
