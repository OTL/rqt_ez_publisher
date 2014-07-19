import os
import rospy
from . import ez_publisher_widget
from . import publisher
from . import config_dialog
from . import quaternion_module
from rqt_py_common.plugin_container_widget import PluginContainerWidget
from qt_gui.plugin import Plugin


class EzPublisherPlugin(Plugin):

    '''Plugin top class for rqt_ez_publisher'''

    def __init__(self, context):
        super(EzPublisherPlugin, self).__init__(context)
        self.setObjectName('EzPublisher')
        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        modules = [quaternion_module.QuaternionModule()]
        self._widget = ez_publisher_widget.EzPublisherWidget(modules=modules)
        self._widget.setObjectName('EzPublisherPluginUi')
        self.mainwidget = PluginContainerWidget(self._widget, True, False)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass
        # self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value(
            'texts', [x.get_text() for x in self._widget.get_sliders()])
        instance_settings.set_value(
            'publish_interval', publisher.TopicPublisherWithTimer.publish_interval)
        for slider in self._widget.get_sliders():
            instance_settings.set_value(
                slider.get_text() + '_range', slider.get_range())
            instance_settings.set_value(
                slider.get_text() + '_is_repeat', slider.is_repeat())

    def restore_settings(self, plugin_settings, instance_settings):
        texts = instance_settings.value('texts')
        if texts:
            for text in texts:
                self._widget.add_slider_by_text(text)
        for slider in self._widget.get_sliders():
            r = instance_settings.value(slider.get_text() + '_range')
            slider.set_range(r)
            is_repeat = instance_settings.value(
                slider.get_text() + '_is_repeat')
            slider.set_is_repeat(is_repeat == 'true')
        interval = instance_settings.value('publish_interval')
        if interval:
            publisher.TopicPublisherWithTimer.publish_interval = int(interval)

    def trigger_configuration(self):
        dialog = config_dialog.ConfigDialog()
        dialog.exec_()
