from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
import value_widget


class DoubleValueWidget(value_widget.ValueWidget):

    LCD_HEIGHT = 35
    DEFAULT_MAX_VALUE = 1.0
    DEFAULT_MIN_VALUE = -1.0
    valueNow = 0.0

    def __init__(self, topic_name, attributes, array_index, publisher, parent,
                 label_text=None):
        self._type = float
        super(DoubleValueWidget, self).__init__(
            topic_name, attributes, array_index, publisher, parent,
            label_text=label_text)

    def set_value(self, value):
        self._lcd.display(value)
        self.publish_value(value)
        self.valueNow = value

    def value_to_slider(self, value):
        return (value - self._min_spin_box.value()) / (
            (self._max_spin_box.value() - self._min_spin_box.value())) * 100

    def slider_to_value(self, val):
        return self._min_spin_box.value() + (
            (self._max_spin_box.value() -
             self._min_spin_box.value()) / 100.0 * val)

    def slider_changed(self, val):
        self.set_value(self.slider_to_value(val))

    def setup_ui(self, name):
        self._min_spin_box = QtWidgets.QDoubleSpinBox()
        self._min_spin_box.setMaximum(10000)
        self._min_spin_box.setMinimum(-10000)
        self._min_spin_box.setValue(self.DEFAULT_MIN_VALUE)
        self._slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self._slider.valueChanged.connect(self.slider_changed)
        self._max_spin_box = QtWidgets.QDoubleSpinBox()
        self._max_spin_box.setMaximum(10000)
        self._max_spin_box.setMinimum(-10000)
        self._max_spin_box.setValue(self.DEFAULT_MAX_VALUE)
        self._lcd = QtWidgets.QLCDNumber()
        self._lcd.setMaximumHeight(self.LCD_HEIGHT)
        self._slider.setValue(50)
        zero_button = QtWidgets.QPushButton('reset')
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
