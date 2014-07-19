from .. import base_module
from . import rpy_widget


class QuaternionModule(base_module.BaseModule):

    def get_msg_string(self):
        return 'geometry_msgs/Quaternion'

    def get_widget_class(self):
        return rpy_widget.RPYWidget
