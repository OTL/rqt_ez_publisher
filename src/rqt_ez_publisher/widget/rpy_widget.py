from .. import ez_publisher_model as ez_model
import math
from python_qt_binding import QtGui
import rpy_value_widget

class RPYWidget(QtGui.QWidget):

    def __init__(self, topic_name, attributes, array_index, publisher,
                 parent=None):
        super(RPYWidget, self).__init__(parent=parent)
        self._attributes = attributes
        self._publisher = publisher
        self._array_index = array_index
        self._topic_name = topic_name
        self._text = ez_model.make_text(topic_name, attributes, array_index)
        self._vertical_layout = QtGui.QVBoxLayout()
        self._widgets = []
        for i in range(3):
            widget = rpy_value_widget.RPYValueWidget(
                topic_name, attributes, array_index, publisher, i, self)
            self._widgets.append(widget)
            self._vertical_layout.addWidget(widget)

        self.set_range([-math.pi, math.pi])
        self.setLayout(self._vertical_layout)
        self.add_button = None

    def get_text(self):
        return self._text

    def get_range(self):
        return self._widgets[0].get_range()

    def set_range(self, r):
        for widget in self._widgets:
            widget.set_range(r)

    def is_repeat(self):
        self._widgets[0].is_repeat()

    def set_is_repeat(self, is_repeat):
        for widget in self._widgets:
            widget.set_is_repeat(is_repeat)

    def get_topic_name(self):
        return self._topic_name

    def update(self):
        for widget in self._widgets:
            widget.update()
