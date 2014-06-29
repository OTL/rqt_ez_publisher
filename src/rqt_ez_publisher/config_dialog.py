from python_qt_binding import QtGui
from python_qt_binding.QtGui import QDialog
from python_qt_binding.QtGui import QDialogButtonBox
from .ez_publisher_widget import TopicPublisherWithTimer

class ConfigDialog(QDialog):

    def __init__(self):
        super(ConfigDialog, self).__init__()
        self._interval_spin_box = QtGui.QSpinBox()
        self._interval_spin_box.setMaximum(10000)
        self._interval_spin_box.setMinimum(1)
        self._interval_spin_box.setValue(TopicPublisherWithTimer.publish_interval)
        self._interval_spin_box.valueChanged.connect(self.update_interval)
        self._horizontal_layout = QtGui.QHBoxLayout()
        spin_label = QtGui.QLabel('Publish Interval for repeat [ms]')
        self._horizontal_layout.addWidget(spin_label)
        self._horizontal_layout.addWidget(self._interval_spin_box)
        self.setLayout(self._horizontal_layout)
        self.adjustSize()

    def update_interval(self, interval):
        TopicPublisherWithTimer.publish_interval = interval
       
