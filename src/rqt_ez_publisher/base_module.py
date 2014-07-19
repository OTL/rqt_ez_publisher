import roslib.message


class BaseModule(object):

    '''Base class for modules for specific message type'''

    def get_msg_string(self):
        return ''

    def get_msg_class(self):
        return roslib.message.get_message_class(self.get_msg_string())

    def get_widget_class(self):
        return None
