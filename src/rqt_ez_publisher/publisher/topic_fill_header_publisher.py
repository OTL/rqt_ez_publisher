import rospy
from . import topic_publisher
import tf2_msgs.msg


class TopicFillHeaderPublisher(topic_publisher.TopicPublisher):

    def __init__(self, topic_name, message_class):
        super(TopicFillHeaderPublisher, self).__init__(
            topic_name, message_class)
        self._is_tf = False
        if message_class == tf2_msgs.msg.TFMessage:
            self._is_tf = True
        self._has_header = False
        if hasattr(self._message, 'header'):
            if hasattr(self._message.header, 'stamp'):
                self._has_header = True

    def publish(self):
        if self._is_tf:
            now = rospy.Time.now()
            for transform in self._message.transforms:
                transform.header.stamp = now
        if self._has_header:
            self._message.header.stamp = rospy.Time.now()
        super(TopicFillHeaderPublisher, self).publish()
