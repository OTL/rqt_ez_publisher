from python_qt_binding import QtWidgets
import value_widget


class StringValueWidget(value_widget.ValueWidget):

    valueNow = ''

    def __init__(self, topic_name, attributes, array_index, publisher, parent):
        self._type = str
        super(StringValueWidget, self).__init__(
            topic_name, attributes, array_index, publisher, parent)

    def input_text(self):
        self.valueNow = str(self._line_edit.text())
        self.publish_value(str(self._line_edit.text()))

    def setup_ui(self, name):
        self._line_edit = QtWidgets.QLineEdit()
        self._line_edit.returnPressed.connect(self.input_text)
        self._horizontal_layout.addWidget(self._line_edit)
        self.setLayout(self._horizontal_layout)
