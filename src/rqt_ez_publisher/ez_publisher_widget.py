import rospy
import sys

from python_qt_binding import QtGui
from python_qt_binding.QtCore import Qt, Signal, QTimer
from .ez_publisher_model import *

LCD_HEIGHT = 35
DEFAULT_PUBLISH_INTERVAL = 100


class TopicPublisherWithTimer(TopicPublisher):

    publish_interval = DEFAULT_PUBLISH_INTERVAL

    def __init__(self, topic_name, message_class):
        TopicPublisher.__init__(self, topic_name, message_class)
        self._timer = None
        self._manager = None

    def update_timer_interval(self, interval):
        if self._timer:
            self._timer.setInterval(interval)

    def set_timer(self, parent=None):
        interval = self.publish_interval
        if not self._timer:
            self._timer = QTimer(parent=parent)
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


class ValueWidget(QtGui.QWidget):

    def __init__(self, topic_name, attributes, array_index, publisher, parent=None):
        QtGui.QWidget.__init__(self, parent=parent)
        self._attributes = attributes
        self._publisher = publisher
        self._array_index = array_index
        self._topic_name = topic_name
        self._text = make_text(topic_name, attributes, array_index)
        self._horizontal_layout = QtGui.QHBoxLayout()
        topic_label = QtGui.QLabel(self._text)
        self.close_button = QtGui.QPushButton()
        self.close_button.setMaximumWidth(30)
        self.close_button.setIcon(self.style().standardIcon(QtGui.QStyle.SP_TitleBarCloseButton))
        self.up_button = QtGui.QPushButton()
        self.up_button.setIcon(self.style().standardIcon(QtGui.QStyle.SP_ArrowUp))
        self.up_button.setMaximumWidth(30)
        self.down_button = QtGui.QPushButton()
        self.down_button.setMaximumWidth(30)
        self.down_button.setIcon(self.style().standardIcon(QtGui.QStyle.SP_ArrowDown))
        repeat_label = QtGui.QLabel('repeat')
        self._repeat_box = QtGui.QCheckBox()
        self._repeat_box.stateChanged.connect(self.repeat_changed)
        self._repeat_box.setChecked(publisher.is_repeating())
        self._horizontal_layout.addWidget(topic_label)
        self._horizontal_layout.addWidget(self.close_button)
        self._horizontal_layout.addWidget(self.up_button)
        self._horizontal_layout.addWidget(self.down_button)
        self._horizontal_layout.addWidget(repeat_label)
        self._horizontal_layout.addWidget(self._repeat_box)
        if self._array_index != None:
            self.add_button = QtGui.QPushButton('+')
            self.add_button.setMaximumWidth(30)
            self._horizontal_layout.addWidget(self.add_button)
        else:
            self.add_button = None
        self.setup_ui(self._text)

    def get_topic_name(self):
        return self._topic_name

    def is_repeat(self):
        return self._publisher.is_repeating()

    def set_is_repeat(self, repeat_on):
        if repeat_on:
            self._publisher.set_timer()
        else:
            self._publisher.stop_timer()
        self._publisher.request_update()

    def repeat_changed(self, state):
        self.set_is_repeat(state == 2)

    def update(self):
        self._repeat_box.setChecked(self._publisher.is_repeating())

    def get_text(self):
        return self._text

    def publish_value(self, value):
        set_msg_attribute_value(self._publisher.get_message(), self._topic_name,
                                self._type, self._attributes, self._array_index, value)
        self._publisher.publish()

    def setup_ui(self, name):
        pass

    def get_range(self):
        return (0, 0)

    def set_range(self, range):
        pass


class BoolValueWidget(ValueWidget):

    def __init__(self, topic_name, attributes, array_index, publisher, parent=None):
        self._type = bool
        ValueWidget.__init__(
            self, topic_name, attributes, array_index, publisher, parent=parent)

    def state_changed(self, state):
        self.publish_value(self._check_box.isChecked())

    def setup_ui(self, name):
        self._check_box = QtGui.QCheckBox()
        self._check_box.stateChanged.connect(self.state_changed)
        self._horizontal_layout.addWidget(self._check_box)
        self.setLayout(self._horizontal_layout)


class StringValueWidget(ValueWidget):

    def __init__(self, topic_name, attributes, array_index, publisher, parent=None):
        self._type = str
        ValueWidget.__init__(
            self, topic_name, attributes, array_index, publisher, parent=parent)

    def input_text(self):
        self.publish_value(str(self._line_edit.text()))

    def setup_ui(self, name):
        self._line_edit = QtGui.QLineEdit()
        self._line_edit.returnPressed.connect(self.input_text)
        self._horizontal_layout.addWidget(self._line_edit)
        self.setLayout(self._horizontal_layout)


class IntValueWidget(ValueWidget):

    def __init__(self, topic_name, attributes, array_index, publisher, parent=None):
        self._type = int
        ValueWidget.__init__(
            self, topic_name, attributes, array_index, publisher, parent=parent)

    def slider_changed(self, value):
        self._lcd.display(value)
        self.publish_value(value)

    def setup_ui(self, name, max_value=100000, min_value=-100000,
                 default_max_value=100, default_min_value=-100,
                 initial_value=0):
        self._min_spin_box = QtGui.QSpinBox()
        self._min_spin_box.setMaximum(max_value)
        self._min_spin_box.setMinimum(min_value)
        self._slider = QtGui.QSlider(Qt.Horizontal)
        self._slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self._slider.valueChanged.connect(self.slider_changed)
        self._max_spin_box = QtGui.QSpinBox()
        self._max_spin_box.setMaximum(max_value)
        self._max_spin_box.setMinimum(min_value)
        self._lcd = QtGui.QLCDNumber()
        self._lcd.setMaximumHeight(LCD_HEIGHT)
        self._min_spin_box.valueChanged.connect(self._slider.setMinimum)
        self._max_spin_box.valueChanged.connect(self._slider.setMaximum)
        self._min_spin_box.setValue(default_min_value)
        self._max_spin_box.setValue(default_max_value)
        self._slider.setValue(initial_value)
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

    def __init__(self, topic_name, attributes, array_index, publisher, parent=None):
        IntValueWidget.__init__(
            self, topic_name, attributes, array_index, publisher, parent=parent)

    def setup_ui(self, name):
        IntValueWidget.setup_ui(self, name, min_value=0, default_min_value=0)


class DoubleValueWidget(ValueWidget):

    DEFAULT_MAX_VALUE = 10.0
    DEFAULT_MIN_VALUE = -10.0

    def __init__(self, topic_name, attributes, array_index, publisher, parent=None):
        self._type = float
        ValueWidget.__init__(
            self, topic_name, attributes, array_index, publisher, parent=parent)

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
        self._min_spin_box.setValue(self.DEFAULT_MIN_VALUE)
        self._slider = QtGui.QSlider(Qt.Horizontal)
        self._slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self._slider.valueChanged.connect(self.slider_changed)
        self._max_spin_box = QtGui.QDoubleSpinBox()
        self._max_spin_box.setMaximum(10000)
        self._max_spin_box.setMinimum(-10000)
        self._max_spin_box.setValue(self.DEFAULT_MAX_VALUE)
        self._lcd = QtGui.QLCDNumber()
        self._lcd.setMaximumHeight(LCD_HEIGHT)
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
    sig_sysmsg = Signal(str)

    def __init__(self, parent=None):
        self._model = EasyPublisherModel(TopicPublisherWithTimer)
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

    def add_widget(self, output_type, topic_name, attributes, array_index, position=None):
        widget_class = None
        type_class_dict = {float: DoubleValueWidget,
                           int: IntValueWidget,
                           'uint': UIntValueWidget,
                           bool: BoolValueWidget,
                           str: StringValueWidget}
        if output_type in type_class_dict:
            widget_class = type_class_dict[output_type]
        else:
            self.sig_sysmsg.emit('not supported type %s' % output_type)
            return False
        widget = widget_class(topic_name, attributes, array_index,
                              self._model.get_publisher(topic_name),
                              parent=self)
        self._model.get_publisher(topic_name).set_manager(self)
        self._sliders.append(widget)
        widget.close_button.clicked.connect(
            lambda x: self.close_slider(widget))
        widget.up_button.clicked.connect(
            lambda x: self.move_up_widget(widget))
        widget.down_button.clicked.connect(
            lambda x: self.move_down_widget(widget))
        if widget.add_button:
            widget.add_button.clicked.connect(
                lambda x: self.add_widget(output_type, topic_name, attributes,
                                          self.get_next_index(output_type, topic_name, attributes),
                                          self._main_vertical_layout.indexOf(widget) + 1))
        if position:
            self._main_vertical_layout.insertWidget(position, widget)
        else:
            self._main_vertical_layout.addWidget(widget)
        return True

    def move_down_widget(self, widget):
        index = self._main_vertical_layout.indexOf(widget)
        if index < self._main_vertical_layout.count() - 1:
            self._main_vertical_layout.removeWidget(widget)
            self._main_vertical_layout.insertWidget(index + 1, widget)

    def move_up_widget(self, widget):
        index = self._main_vertical_layout.indexOf(widget)
        if index > 1:
            self._main_vertical_layout.removeWidget(widget)
            self._main_vertical_layout.insertWidget(index - 1, widget)
        

    def add_slider_by_text(self, text):
        if text in [x.get_text() for x in self._sliders]:
            self.sig_sysmsg.emit('%s is already exists' % text)
            return
        results = self._model.resister_topic_by_text(text)
        if not results:
            self.sig_sysmsg.emit('%s does not exists' % text)
            return
        topic_name, attributes, builtin_type, is_array, array_index = results
        if builtin_type:
            if is_array and array_index == None:
                # use index 0
                array_index = 0
            self.add_widget(builtin_type, topic_name, attributes, array_index)
        else:
            for string in self._model.expand_attribute(text, array_index):
                self.add_slider_by_text(string)

    def get_sliders_for_topic(self, topic):
        return [x for x in self._sliders if x.get_topic_name() == topic]

    def get_sliders(self):
        return self._sliders

    def clear_sliders(self):
        for widget in self._sliders:
            self.close_slider(widget, False)
        self._sliders = []

    def update_combo_items(self):
        self._combo.clear()
        for topic in self._model.get_topic_names():
            self._combo.addItem(topic)

    def setup_ui(self):
        horizontal_layout = QtGui.QHBoxLayout()
        reload_button = QtGui.QPushButton(parent=self)
        reload_button.setMaximumWidth(30)
        reload_button.setIcon(self.style().standardIcon(QtGui.QStyle.SP_BrowserReload))
        reload_button.clicked.connect(self.update_combo_items)
        topic_label = QtGui.QLabel('topic(+data member) name')
        clear_button = QtGui.QPushButton('all clear')
        clear_button.setMaximumWidth(200)
        clear_button.clicked.connect(self.clear_sliders)
        self._combo = QtGui.QComboBox()
        self._combo.setEditable(True)
        self.update_combo_items()
        self._combo.activated.connect(self.add_slider_from_combo)
        horizontal_layout.addWidget(reload_button)
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
