from python_qt_binding.QtWidgets import QWidget


class BaseWidget(QWidget):

    def __init__(self, topic_name, publisher, parent=None):
        super(BaseWidget, self).__init__(parent=None)
        self._topic_name = topic_name
        self._publisher = publisher

    def get_text(self):
        return ''

    def get_range(self):
        return (0, 0)

    def set_range(self, r):
        pass

    def is_repeat(self):
        return self._publisher.is_repeating()

    def set_is_repeat(self, repeat_on):
        if repeat_on:
            self._publisher.set_timer()
        else:
            self._publisher.stop_timer()
        self._publisher.request_update()

    def get_topic_name(self):
        return self._topic_name

    def update(self):
        pass

