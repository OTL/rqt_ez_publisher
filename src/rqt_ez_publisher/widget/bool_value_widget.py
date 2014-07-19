from python_qt_binding import QtGui
import value_widget


class BoolValueWidget(value_widget.ValueWidget):

    def __init__(self, topic_name, attributes, array_index, publisher, parent):
        self._type = bool
        super(BoolValueWidget, self).__init__(
            topic_name, attributes, array_index, publisher, parent)

    def state_changed(self, state):
        self.publish_value(self._check_box.isChecked())

    def setup_ui(self, name):
        self._check_box = QtGui.QCheckBox()
        self._check_box.stateChanged.connect(self.state_changed)
        self._horizontal_layout.addWidget(self._check_box)
        self.setLayout(self._horizontal_layout)
