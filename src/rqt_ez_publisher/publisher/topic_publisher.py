import rospy

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
