import rospy
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets
from python_qt_binding.QtWidgets import QWidget
from rqt_ez_publisher import ez_publisher_model as ez_model
from rqt_ez_publisher import widget as ez_widget
from rqt_ez_publisher import publisher


class EzPublisherWidget(QWidget):

    '''Main widget of this GUI'''

    sig_sysmsg = QtCore.Signal(str)

    def __init__(self, parent=None, modules=[]):
        self._model = ez_model.EzPublisherModel(
            publisher.TopicPublisherWithTimer, modules=modules)
        self._sliders = []
        QWidget.__init__(self, parent=parent)
        self.setup_ui()

    def add_slider_from_combo(self):
        return self.add_slider_by_text(str(self._combo.currentText()))

    def close_slider(self, widget, remove=True):
        widget.hide()
        if remove:
            self._sliders.remove(widget)
        self._main_vertical_layout.removeWidget(widget)

    def get_next_index(self, topic_name, attributes):
        array_index = 0
        text = ez_model.make_text(topic_name, attributes, array_index)
        while text in [x.get_text() for x in self._sliders]:
            array_index += 1
            text = ez_model.make_text(topic_name, attributes, array_index)
        return array_index

    def add_widget(self, output_type, topic_name, attributes, array_index,
                   position=None):
        widget_class = None
        type_class_dict = {float: ez_widget.DoubleValueWidget,
                           int: ez_widget.IntValueWidget,
                           'uint': ez_widget.UIntValueWidget,
                           bool: ez_widget.BoolValueWidget,
                           str: ez_widget.StringValueWidget}
        for module in self._model.get_modules():
            type_class_dict[
                module.get_msg_string()] = module.get_widget_class()
        if output_type in type_class_dict:
            widget_class = type_class_dict[output_type]
        else:
            self.sig_sysmsg.emit('not supported type %s' % output_type)
            return False
        widget = widget_class(topic_name, attributes, array_index,
                              self._model.get_publisher(topic_name), self)
        self._model.get_publisher(topic_name).set_manager(self)
        self._sliders.append(widget)
        if widget.add_button:
            widget.add_button.clicked.connect(
                lambda: self.add_widget(
                    output_type, topic_name, attributes,
                    self.get_next_index(topic_name, attributes),
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
        if text.endswith('/header/seq'):
            rospy.loginfo('header/seq is not created')
            return
        if text in [x.get_text() for x in self._sliders]:
            self.sig_sysmsg.emit('%s is already exists' % text)
            return
        results = self._model.register_topic_by_text(text)
        if not results:
            self.sig_sysmsg.emit('%s does not exists' % text)
            return
        topic_name, attributes, builtin_type, is_array, array_index = results
        if builtin_type:
            if is_array and array_index is None:
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
        horizontal_layout = QtWidgets.QHBoxLayout()
        reload_button = QtWidgets.QPushButton(parent=self)
        reload_button.setMaximumWidth(30)
        reload_button.setIcon(
            self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        reload_button.clicked.connect(self.update_combo_items)
        topic_label = QtWidgets.QLabel('topic(+data member) name')
        clear_button = QtWidgets.QPushButton('all clear')
        clear_button.setMaximumWidth(200)
        clear_button.clicked.connect(self.clear_sliders)
        self._combo = QtWidgets.QComboBox()
        self._combo.setEditable(True)
        self.update_combo_items()
        self._combo.activated.connect(self.add_slider_from_combo)
        horizontal_layout.addWidget(reload_button)
        horizontal_layout.addWidget(topic_label)
        horizontal_layout.addWidget(self._combo)
        horizontal_layout.addWidget(clear_button)
        self._main_vertical_layout = QtWidgets.QVBoxLayout()
        self._main_vertical_layout.addLayout(horizontal_layout)
        self._main_vertical_layout.setAlignment(
            horizontal_layout, QtCore.Qt.AlignTop)
        self.setLayout(self._main_vertical_layout)

    def shutdown(self):
        self._model.shutdown()


def main():
    import sys
    app = QtWidgets.QApplication(sys.argv)
    main_window = QtWidgets.QMainWindow()
    main_widget = EzPublisherWidget()
    main_window.setCentralWidget(main_widget)
    main_window.show()
    app.exec_()

if __name__ == '__main__':
    rospy.init_node('ez_publisher')
    main()
