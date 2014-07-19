from python_qt_binding import QtGui
from python_qt_binding.QtGui import QDialog
from python_qt_binding.QtGui import QDialogButtonBox
from . import publisher


class ConfigDialog(QDialog):

    '''Dialog for configure button of rqt system

    - set time interval for repeated publishing'''

    def __init__(self):
        super(ConfigDialog, self).__init__()
        self._interval_spin_box = QtGui.QSpinBox()
        self._interval_spin_box.setMaximum(10000)
        self._interval_spin_box.setMinimum(1)
        self._interval_spin_box.setValue(
            publisher.TopicPublisherWithTimer.publish_interval)
        self._interval_spin_box.valueChanged.connect(self.update_interval)
        self._horizontal_layout = QtGui.QHBoxLayout()
        spin_label = QtGui.QLabel('Publish Interval for repeat [ms]')
        self._horizontal_layout.addWidget(spin_label)
        self._horizontal_layout.addWidget(self._interval_spin_box)
        self.setLayout(self._horizontal_layout)
        self.adjustSize()

    def update_interval(self, interval):
        publisher.TopicPublisherWithTimer.publish_interval = interval
