import os
import rospy
from rqt_ez_publisher.ez_publisher_widget import EasyPublisherWidget
from qt_gui.plugin import Plugin

class EzPublisherPlugin(Plugin):

    def __init__(self, context):
        super(EzPublisherPlugin, self).__init__(context)
        self.setObjectName('EzPublisher')
        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        # Create QWidget
        self._widget = EasyPublisherWidget()
        self._widget.setObjectName('EzPublisherPluginUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass
        #self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('texts', [x.get_text() for x in self._widget.get_sliders()])
        for slider in self._widget.get_sliders():
            instance_settings.set_value(
                slider.get_text() + '_range', slider.get_range())

    def restore_settings(self, plugin_settings, instance_settings):
        texts = instance_settings.value('texts')
        if texts:
            for text in texts:
                self._widget.add_slider_by_text(text)
        for slider in self._widget.get_sliders():
            r = instance_settings.value(slider.get_text() + '_range')
            slider.set_range(r)


    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

