import rospy
import roslib.message
import sys

from python_qt_binding import QtGui
from python_qt_binding.QtCore import Qt

def make_topic_strings(t, string=''):
    try:
        return [make_topic_strings(t.__getattribute__(slot), string + '/' + slot) for slot in t.__slots__]
    except AttributeError, e:
        return string

def flatten(L):
    if isinstance(L, list):
       return reduce(lambda a,b: a + flatten(b), L, [])
    else:
        return [L]


def find_topic_name(full_text, topic_dict):
    if full_text[0] != '/':
        full_text = '/' + full_text
    # This is topic
    if full_text in topic_dict:
        return (full_text, None)
    splited_text = full_text.split('/')[1:]
    topic_name = ''
    while not topic_name in topic_dict and splited_text:
        topic_name += '/' + splited_text[0]
        splited_text = splited_text[1:]
    if splited_text:
        return (topic_name, splited_text)
    else:
        return (None, None)

        
class ValueWidget(QtGui.QWidget):
    def __init__(self, topic_name, attributes, message, publisher, parent=None):
        QtGui.QWidget.__init__(self, parent=parent)
        self._attributes = attributes
        self._publisher = publisher
        self._message = message
        self._text = topic_name + '/' + '/'.join(attributes)
        self.setup_ui(self._text)

    def get_text(self):
        return self._text

    def publish_value(self, value):
        message_target = self._message
        for attr in self._attributes[:-1]:
            message_target = message_target.__getattribute__(attr)
        message_target.__setattr__(self._attributes[-1], value)
        self._publisher.publish(self._message)

    def setup_ui(self, name):
        pass

    def get_range(self):
        return (0, 0)

    def set_range(self, range):
        pass


class BoolValueWidget(ValueWidget):
    def __init__(self, topic_name, attributes, message, publisher, parent=None):
        ValueWidget.__init__(self, topic_name, attributes, message, publisher, parent=parent)

    def state_changed(self, state):
        self.publish_value(self._check_box.isChecked())

    def setup_ui(self, name):
        self._horizontal_layout = QtGui.QHBoxLayout()
        topic_label = QtGui.QLabel(name)
        self._check_box = QtGui.QCheckBox()
        self._check_box.stateChanged.connect(self.state_changed)
        self.close_button = QtGui.QPushButton('x')
        self.close_button.setMaximumWidth(30)
        self._horizontal_layout.addWidget(self.close_button)
        self._horizontal_layout.addWidget(topic_label)
        self._horizontal_layout.addWidget(self._check_box)
        self.setLayout(self._horizontal_layout)


class StringValueWidget(ValueWidget):
    def __init__(self, topic_name, attributes, message, publisher, parent=None):
        ValueWidget.__init__(self, topic_name, attributes, message, publisher, parent=parent)

    def input_text(self):
        self.publish_value(str(self._line_edit.text()))

    def setup_ui(self, name):
        self._horizontal_layout = QtGui.QHBoxLayout()
        topic_label = QtGui.QLabel(name)
        self._line_edit = QtGui.QLineEdit()
        self._line_edit.returnPressed.connect(self.input_text)
        self.close_button = QtGui.QPushButton('x')
        self.close_button.setMaximumWidth(30)
        self._horizontal_layout.addWidget(self.close_button)
        self._horizontal_layout.addWidget(topic_label)
        self._horizontal_layout.addWidget(self._line_edit)
        self.setLayout(self._horizontal_layout)


class IntValueWidget(ValueWidget):
    def __init__(self, topic_name, attributes, message, publisher, parent=None):
        ValueWidget.__init__(self, topic_name, attributes, message, publisher, parent=parent)

    def slider_changed(self, value):
        self._lcd.display(value)
        self.publish_value(value)

    def setup_ui(self, name):
        self._horizontal_layout = QtGui.QHBoxLayout()
        topic_label = QtGui.QLabel(name)
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
        self.close_button = QtGui.QPushButton('x')
        self.close_button.setMaximumWidth(30)
        self._horizontal_layout.addWidget(self.close_button)
        self._horizontal_layout.addWidget(topic_label)
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


class DoubleValueWidget(ValueWidget):
    def __init__(self, topic_name, attributes, message, publisher, parent=None):
        ValueWidget.__init__(self, topic_name, attributes, message, publisher, parent=parent)

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
        self._horizontal_layout = QtGui.QHBoxLayout()
        topic_label = QtGui.QLabel(name)
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
        zero_button.clicked.connect(lambda x: self._slider.setValue(self.value_to_slider(0.0)))
        self.close_button = QtGui.QPushButton('x')
        self.close_button.setMaximumWidth(30)
        self._horizontal_layout.addWidget(self.close_button)
        self._horizontal_layout.addWidget(topic_label)
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
        QtGui.QWidget.__init__(self, parent=parent)
        self.setup_ui()
        self._sliders = []
        # model
        self._publishers = {}
        self._messages = {}
        
    def add_slider(self):
        self.add_slider_by_text(str(self._line_edit.text()))

    def close_slider(self, widget):
        widget.hide()
        self._sliders.remove(widget)
        self._main_vertical_layout.removeWidget(widget)

    def add_widget(self, output_type, topic_name, attributes):
        widget_class = None
        type_class_dict = {float: DoubleValueWidget,
                           int: IntValueWidget,
                           bool: BoolValueWidget,
                           str: StringValueWidget}
        if output_type in type_class_dict:
            widget_class = type_class_dict[output_type]
        else:
            rospy.logerr('not supported type %s'%output_type)
            return False
        widget = widget_class(topic_name, attributes, self._messages[topic_name], self._publishers[topic_name])
        self._sliders.append(widget)
        widget.close_button.clicked.connect(lambda x: self.close_slider(widget))
        self._main_vertical_layout.addWidget(widget)
        return True
        
    def add_slider_by_text(self, text):
        _, _, topic_types = rospy.get_master().getTopicTypes()
        topic_dict = dict(topic_types)
        topic_name, attributes = find_topic_name(text, topic_dict)
        if not topic_name:
            rospy.logerr('%s not found'%text)
            return
        topic_type_str = topic_dict[topic_name]
        message_class = roslib.message.get_message_class(topic_type_str)
        if not topic_name in self._publishers:
            self._publishers[topic_name] = rospy.Publisher(topic_name, message_class)
        if not topic_name in self._messages:
            self._messages[topic_name] = message_class()
        if not attributes:
            for break_down_string in flatten(make_topic_strings(self._messages[topic_name], topic_name)):
                self.add_slider_by_text(break_down_string)
        else:
            message_target = self._messages[topic_name]
            try:
                for attr in attributes:
                    message_target = message_target.__getattribute__(attr)
            except AttributeError, e:
                rospy.logerr(e)
                return
            self.add_widget(type(message_target), topic_name, attributes)

    def get_sliders(self):
        return self._sliders
        
    def clear_sliders(self):
        for widget in self._sliders:
            self.close_slider(widget)

    def setup_ui(self):
        horizontal_layout = QtGui.QHBoxLayout()
        topic_label = QtGui.QLabel('topic/data (rqt_plot like)')
        horizontal_layout.addWidget(topic_label)
        self._line_edit = QtGui.QLineEdit()
        self._line_edit.returnPressed.connect(self.add_slider)
        horizontal_layout.addWidget(self._line_edit)
        clear_button = QtGui.QPushButton('all clear')
        horizontal_layout.addWidget(clear_button)
        clear_button.clicked.connect(self.clear_sliders)
        self._main_vertical_layout = QtGui.QVBoxLayout()
        self._main_vertical_layout.addLayout(horizontal_layout)
        self._main_vertical_layout.setAlignment(horizontal_layout, Qt.AlignTop)
        self.setLayout(self._main_vertical_layout)

    def shutdown(self):
        for pub in self._publishers.values():
            pub.unregister()


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
