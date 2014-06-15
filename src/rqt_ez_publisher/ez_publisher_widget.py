import rospy
import sys

from python_qt_binding import QtGui
from python_qt_binding.QtCore import Qt
from rqt_ez_publisher.ez_publisher_model import *


class ValueWidget(QtGui.QWidget):

    def __init__(self, topic_name, attributes, array_index, message, publisher, parent=None):
        QtGui.QWidget.__init__(self, parent=parent)
        self._attributes = attributes
        self._publisher = publisher
        self._message = message
        self._array_index = array_index
        self._text = make_text(topic_name, attributes, array_index)
        self._horizontal_layout = QtGui.QHBoxLayout()
        topic_label = QtGui.QLabel(self._text)
        self.close_button = QtGui.QPushButton('x')
        self.close_button.setMaximumWidth(30)
        self._horizontal_layout.addWidget(self.close_button)
        self._horizontal_layout.addWidget(topic_label)
        if self._array_index != None:
            self.add_button = QtGui.QPushButton('+')
            self.add_button.setMaximumWidth(30)
            self._horizontal_layout.addWidget(self.add_button)
        else:
            self.add_button = None
        self.setup_ui(self._text)

    def get_text(self):
        return self._text

    def publish_value(self, value):
        message_target = self._message
        if len(self._attributes) >= 2:
            for attr in self._attributes[:-1]:
                message_target = message_target.__getattribute__(attr)
        if self._array_index != None:
            array = message_target.__getattribute__(self._attributes[-1])
            while len(array) <= self._array_index:
                array.append(self._type())
            array[self._array_index] = value
            message_target.__setattr__(self._attributes[-1], array)
        else:
            message_target.__setattr__(self._attributes[-1], value)
        self._publisher.publish(self._message)

    def setup_ui(self, name):
        pass

    def get_range(self):
        return (0, 0)

    def set_range(self, range):
        pass


class BoolValueWidget(ValueWidget):

    def __init__(self, topic_name, attributes, array_index, message, publisher, parent=None):
        self._type = bool
        ValueWidget.__init__(
            self, topic_name, attributes, array_index, message, publisher, parent=parent)

    def state_changed(self, state):
        self.publish_value(self._check_box.isChecked())

    def setup_ui(self, name):
        self._check_box = QtGui.QCheckBox()
        self._check_box.stateChanged.connect(self.state_changed)
        self._horizontal_layout.addWidget(self._check_box)
        self.setLayout(self._horizontal_layout)


class StringValueWidget(ValueWidget):

    def __init__(self, topic_name, attributes, array_index, message, publisher, parent=None):
        self._type = str
        ValueWidget.__init__(
            self, topic_name, attributes, array_index, message, publisher, parent=parent)

    def input_text(self):
        self.publish_value(str(self._line_edit.text()))

    def setup_ui(self, name):
        self._line_edit = QtGui.QLineEdit()
        self._line_edit.returnPressed.connect(self.input_text)
        self._horizontal_layout.addWidget(self._line_edit)
        self.setLayout(self._horizontal_layout)


class IntValueWidget(ValueWidget):

    def __init__(self, topic_name, attributes, array_index, message, publisher, parent=None):
        self._type = int
        ValueWidget.__init__(
            self, topic_name, attributes, array_index, message, publisher, parent=parent)

    def slider_changed(self, value):
        self._lcd.display(value)
        self.publish_value(value)

    def setup_ui(self, name):
        self._min_spin_box = QtGui.QSpinBox()
        self._min_spin_box.setMaximum(10000)
        self._min_spin_box.setMinimum(-10000)
        self._slider = QtGui.QSlider(Qt.Horizontal)
        self._slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self._slider.valueChanged.connect(self.slider_changed)
        self._max_spin_box = QtGui.QSpinBox()
        self._max_spin_box.setMaximum(10000)
        self._max_spin_box.setMinimum(-10000)
        self._lcd = QtGui.QLCDNumber()
        self._min_spin_box.valueChanged.connect(self._slider.setMinimum)
        self._max_spin_box.valueChanged.connect(self._slider.setMaximum)
        self._min_spin_box.setValue(-10)
        self._max_spin_box.setValue(10)
        self._slider.setValue(0)
        zero_button = QtGui.QPushButton('reset')
        zero_button.clicked.connect(lambda x: self._slider.setValue(0))
        self._horizontal_layout.addWidget(self._min_spin_box)
        self._horizontal_layout.addWidget(self._slider)
        self._horizontal_layout.addWidget(self._max_spin_box)
        self._horizontal_layout.addWidget(self._lcd)
        self._horizontal_layout.addWidget(zero_button)

        self.setLayout(self._horizontal_layout)

    def get_range(self):
        return (self._min_spin_box.value(), self._max_spin_box.value())

    def set_range(self, r):
        self._min_spin_box.setValue(r[0])
        self._max_spin_box.setValue(r[1])


class UIntValueWidget(IntValueWidget):

    def __init__(self, topic_name, attributes, array_index, message, publisher, parent=None):
        self._type = int
        ValueWidget.__init__(
            self, topic_name, attributes, array_index, message, publisher, parent=parent)

    def setup_ui(self, name):
        self._min_spin_box = QtGui.QSpinBox()
        self._min_spin_box.setMaximum(10000)
        self._min_spin_box.setMinimum(0)
        self._slider = QtGui.QSlider(Qt.Horizontal)
        self._slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self._slider.valueChanged.connect(self.slider_changed)
        self._max_spin_box = QtGui.QSpinBox()
        self._max_spin_box.setMaximum(10000)
        self._max_spin_box.setMinimum(0)
        self._lcd = QtGui.QLCDNumber()
        self._min_spin_box.valueChanged.connect(self._slider.setMinimum)
        self._max_spin_box.valueChanged.connect(self._slider.setMaximum)
        self._min_spin_box.setValue(0)
        self._max_spin_box.setValue(10)
        self._slider.setValue(0)
        zero_button = QtGui.QPushButton('reset')
        zero_button.clicked.connect(lambda x: self._slider.setValue(0))
        self._horizontal_layout.addWidget(self._min_spin_box)
        self._horizontal_layout.addWidget(self._slider)
        self._horizontal_layout.addWidget(self._max_spin_box)
        self._horizontal_layout.addWidget(self._lcd)
        self._horizontal_layout.addWidget(zero_button)

        self.setLayout(self._horizontal_layout)


class DoubleValueWidget(ValueWidget):

    def __init__(self, topic_name, attributes, array_index, message, publisher, parent=None):
        self._type = float
        ValueWidget.__init__(
            self, topic_name, attributes, array_index, message, publisher, parent=parent)

    def set_value(self, value):
        self._lcd.display(value)
        self.publish_value(value)

    def value_to_slider(self, value):
        return (value - self._min_spin_box.value()) / (self._max_spin_box.value() - self._min_spin_box.value()) * 100

    def slider_to_value(self, val):
        return self._min_spin_box.value() + (self._max_spin_box.value() - self._min_spin_box.value()) / 100.0 * val

    def slider_changed(self, val):
        self.set_value(self.slider_to_value(val))

    def setup_ui(self, name):
        self._min_spin_box = QtGui.QDoubleSpinBox()
        self._min_spin_box.setMaximum(10000)
        self._min_spin_box.setMinimum(-10000)
        self._min_spin_box.setValue(-1.0)
        self._slider = QtGui.QSlider(Qt.Horizontal)
        self._slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self._slider.valueChanged.connect(self.slider_changed)
        self._max_spin_box = QtGui.QDoubleSpinBox()
        self._max_spin_box.setMaximum(10000)
        self._max_spin_box.setMinimum(-10000)
        self._max_spin_box.setValue(1.0)
        self._lcd = QtGui.QLCDNumber()
        self._slider.setValue(50)
        zero_button = QtGui.QPushButton('reset')
        zero_button.clicked.connect(
            lambda x: self._slider.setValue(self.value_to_slider(0.0)))
        self._horizontal_layout.addWidget(self._min_spin_box)
        self._horizontal_layout.addWidget(self._slider)
        self._horizontal_layout.addWidget(self._max_spin_box)
        self._horizontal_layout.addWidget(self._lcd)
        self._horizontal_layout.addWidget(zero_button)

        self.setLayout(self._horizontal_layout)

    def get_range(self):
        return (self._min_spin_box.value(), self._max_spin_box.value())

    def set_range(self, min_max):
        self._min_spin_box.setValue(min_max[0])
        self._max_spin_box.setValue(min_max[1])


class EasyPublisherWidget(QtGui.QWidget):

    def __init__(self, parent=None):
        self._model = EasyPublisherModel()
        self._sliders = []
        QtGui.QWidget.__init__(self, parent=parent)
        self.setup_ui()


    def add_slider(self):
        self.add_slider_by_text(str(self._line_edit.text()))

    def add_slider_from_combo(self):
        self.add_slider_by_text(str(self._combo.currentText()))

    def close_slider(self, widget, remove=True):
        widget.hide()
        if remove:
            self._sliders.remove(widget)
        self._main_vertical_layout.removeWidget(widget)

    def get_next_index(self, output_type, topic_name, attributes):
        array_index = 0
        text = make_text(topic_name, attributes, array_index)
        while text in [x.get_text() for x in self._sliders]:
            array_index += 1
            text = make_text(topic_name, attributes, array_index)
        return array_index

    def add_widget(self, output_type, topic_name, attributes, array_index):
        widget_class = None
        type_class_dict = {float: DoubleValueWidget,
                           int: IntValueWidget,
                           'uint': UIntValueWidget,
                           bool: BoolValueWidget,
                           str: StringValueWidget}
        if output_type in type_class_dict:
            widget_class = type_class_dict[output_type]
        else:
            rospy.logerr('not supported type %s' % output_type)
            return False
        widget = widget_class(topic_name, attributes, array_index,
                              self._model.get_message(topic_name),
                              self._model.get_publisher(topic_name))
                              
        self._sliders.append(widget)
        widget.close_button.clicked.connect(
            lambda x: self.close_slider(widget))
        if widget.add_button:
            widget.add_button.clicked.connect(
                lambda x: self.add_widget(output_type, topic_name, attributes, self.get_next_index(output_type, topic_name, attributes)))
        self._main_vertical_layout.addWidget(widget)
        return True

    def add_slider_by_text(self, text):
        if text in [x.get_text() for x in self._sliders]:
            rospy.logwarn('%s is already exists' % text)
            return
        topic_name, attributes, builtin_type, is_array, array_index = self._model.resister_topic_by_text(text)
        if not attributes:
            for break_down_string in self._model.get_topic_break_down_strings(topic_name):
                self.add_slider_by_text(break_down_string)
            return
        if builtin_type:
            if is_array and array_index == None:
                # use index 0
                array_index = 0
            self.add_widget(builtin_type, topic_name, attributes, array_index)

    def get_sliders(self):
        return self._sliders

    def clear_sliders(self):
        for widget in self._sliders:
            self.close_slider(widget, False)
        self._sliders = []

    def setup_ui(self):
        horizontal_layout = QtGui.QHBoxLayout()
        topic_label = QtGui.QLabel('topic/data')
        clear_button = QtGui.QPushButton('all clear')
        clear_button.clicked.connect(self.clear_sliders)
        self._combo = QtGui.QComboBox()
        self._combo.setEditable(True)
        for topic in self._model.get_topic_names():
            self._combo.addItem(topic)
        self._combo.activated.connect(self.add_slider_from_combo)
        horizontal_layout.addWidget(topic_label)
        horizontal_layout.addWidget(self._combo)
        horizontal_layout.addWidget(clear_button)
        self._main_vertical_layout = QtGui.QVBoxLayout()
        self._main_vertical_layout.addLayout(horizontal_layout)
        self._main_vertical_layout.setAlignment(horizontal_layout, Qt.AlignTop)
        self.setLayout(self._main_vertical_layout)

    def shutdown(self):
        self._model.shutdown()


def main():
    app = QtGui.QApplication(sys.argv)
    main_window = QtGui.QMainWindow()
    main_widget = EasyPublisherWidget()
    main_window.setCentralWidget(main_widget)
    main_window.show()
    app.exec_()

if __name__ == '__main__':
    rospy.init_node('ez_publisher')
    main()
