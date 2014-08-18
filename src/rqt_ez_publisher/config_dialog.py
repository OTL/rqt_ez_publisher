from python_qt_binding import QtGui
from python_qt_binding.QtGui import QDialog
from python_qt_binding.QtGui import QDialogButtonBox
from . import publisher


class ConfigDialog(QDialog):

    '''Dialog for configure button of rqt system

    - set time interval for repeated publishing'''

    def __init__(self, plugin):
        super(ConfigDialog, self).__init__()
        self._plugin = plugin
        self._interval_spin_box = QtGui.QSpinBox()
        self._interval_spin_box.setMaximum(10000)
        self._interval_spin_box.setMinimum(1)
        self._interval_spin_box.setValue(
            publisher.TopicPublisherWithTimer.publish_interval)
        self._interval_spin_box.valueChanged.connect(self.update_interval)
        self._vertical_layout = QtGui.QVBoxLayout()
        self._horizontal_layout = QtGui.QHBoxLayout()
        spin_label = QtGui.QLabel('Publish Interval for repeat [ms]')
        self._horizontal_layout.addWidget(spin_label)
        self._horizontal_layout.addWidget(self._interval_spin_box)
        self._vertical_layout.addLayout(self._horizontal_layout)
        save_button = QtGui.QPushButton(parent=self)
        save_button.setIcon(
            self.style().standardIcon(QtGui.QStyle.SP_DialogSaveButton))
        save_button.setText('Save to file')
        save_button.clicked.connect(self.save_to_file)

        load_button = QtGui.QPushButton(parent=self)
        load_button.setIcon(
            self.style().standardIcon(QtGui.QStyle.SP_DialogOpenButton))
        load_button.setText('Load from file')
        load_button.clicked.connect(self.load_from_file)

        self._vertical_layout.addWidget(save_button)
        self._vertical_layout.addWidget(load_button)
        self.setLayout(self._vertical_layout)
        self.adjustSize()

    def save_to_file(self):
        file_path, _ = QtGui.QFileDialog.getSaveFileNameAndFilter(
            self, 'Open file to save', filter="Setting File (*.yaml)")
        if file_path:
            self._plugin.save_to_file(file_path)
        self.close()

    def load_from_file(self):
        file_path, _ = QtGui.QFileDialog.getOpenFileName(
            self, 'Open file to load', filter="Setting File (*.yaml)")
        if file_path:
            self._plugin.load_from_file(file_path)
        self.close()

    def update_interval(self, interval):
        publisher.TopicPublisherWithTimer.publish_interval = interval
