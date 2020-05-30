from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from . import value_widget


class IntValueWidget(value_widget.ValueWidget):

    LCD_HEIGHT = 35

    def __init__(self, topic_name, attributes, array_index, publisher, parent):
        self._type = int
        super(IntValueWidget, self).__init__(
            topic_name, attributes, array_index, publisher, parent)

    def slider_changed(self, value):
        self._lcd.display(value)
        self.publish_value(value)

    def setup_ui(self, name, max_value=100000, min_value=-100000,
                 default_max_value=100, default_min_value=-100,
                 initial_value=0):
        self._min_spin_box = QtWidgets.QSpinBox()
        self._min_spin_box.setMaximum(max_value)
        self._min_spin_box.setMinimum(min_value)
        self._slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self._slider.valueChanged.connect(self.slider_changed)
        self._max_spin_box = QtWidgets.QSpinBox()
        self._max_spin_box.setMaximum(max_value)
        self._max_spin_box.setMinimum(min_value)
        self._lcd = QtWidgets.QLCDNumber()
        self._lcd.setMaximumHeight(self.LCD_HEIGHT)
        self._min_spin_box.valueChanged.connect(self._slider.setMinimum)
        self._max_spin_box.valueChanged.connect(self._slider.setMaximum)
        self._min_spin_box.setValue(default_min_value)
        self._max_spin_box.setValue(default_max_value)
        self._slider.setValue(initial_value)
        zero_button = QtWidgets.QPushButton('reset')
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
