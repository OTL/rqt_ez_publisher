from . import int_value_widget


class UIntValueWidget(int_value_widget.IntValueWidget):

    def __init__(self, topic_name, attributes, array_index, publisher, parent):
        super(UIntValueWidget, self).__init__(
            topic_name, attributes, array_index, publisher, parent)

    def setup_ui(self, name):
        super(UIntValueWidget, self).setup_ui(
            name, min_value=0, default_min_value=0)
