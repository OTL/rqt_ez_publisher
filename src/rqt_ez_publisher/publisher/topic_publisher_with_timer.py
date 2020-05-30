from . import topic_fill_header_publisher
from python_qt_binding import QtCore

DEFAULT_PUBLISH_INTERVAL = 100


class TopicPublisherWithTimer(topic_fill_header_publisher.TopicFillHeaderPublisher):

    publish_interval = DEFAULT_PUBLISH_INTERVAL

    def __init__(self, topic_name, message_class):
        super(TopicPublisherWithTimer, self).__init__(
            topic_name, message_class)
        self._timer = None
        self._manager = None

    def update_timer_interval(self, interval):
        if self._timer:
            self._timer.setInterval(interval)

    def set_timer(self, parent=None):
        interval = self.publish_interval
        if not self._timer:
            self._timer = QtCore.QTimer(parent=parent)
            self._timer.timeout.connect(self.publish)
        self._timer.setInterval(interval)
        self._timer.start()

    def stop_timer(self):
        if self._timer:
            self._timer.stop()
            self._timer = None

    def is_repeating(self):
        if self._timer and self._timer.isActive():
            return True
        else:
            return False

    def set_manager(self, manager):
        self._manager = manager

    def get_manager(self):
        return self._manager

    def request_update(self):
        for slider in self._manager.get_sliders_for_topic(
                self.get_topic_name()):
            slider.update()
