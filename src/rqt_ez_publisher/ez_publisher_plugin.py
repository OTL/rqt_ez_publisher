import os
import rospy
import yaml
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
        modules = [quaternion_module.QuaternionModule()]
        self._widget = ez_publisher_widget.EzPublisherWidget(modules=modules)
        self._widget.setObjectName('EzPublisherPluginUi')
        self.mainwidget = PluginContainerWidget(self._widget, True, False)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        from argparse import ArgumentParser
        parser = ArgumentParser(prog='rqt_ez_publisher')
        EzPublisherPlugin.add_arguments(parser)
        args, unknowns = parser.parse_known_args(context.argv())
        self._loaded_settings = None
        if args.slider_file is not None:
            self.load_from_file(args.slider_file)

    def shutdown_plugin(self):
        pass

    def save_to_file(self, file_path):
        try:
            f = open(file_path, 'w')
            f.write(yaml.safe_dump(self.save_to_dict(),
                                   encoding='utf-8', allow_unicode=True))
            f.close()
            rospy.loginfo('saved as %s' % file_path)
        except IOError as e:
            rospy.logerr('%s: Failed to save as %s' % (e, file_path))

    def load_from_file(self, file_path):
        if os.path.exists(file_path):
            self._widget.clear_sliders()
            self._loaded_settings = yaml.load(open(file_path).read())
            self.restore_from_dict(self._loaded_settings)
        else:
            rospy.logerr('%s is not found' % file_path)

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
        if self._loaded_settings is not None:
            return
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

    def restore_from_dict(self, settings):
        for text in settings['texts']:
            self._widget.add_slider_by_text(text)
        for slider in self._widget.get_sliders():
            try:
                slider_setting = settings['settings'][slider.get_text()]
                slider.set_range([slider_setting['min'], slider_setting['max']])

                slider.set_is_repeat(slider_setting['is_repeat'])
            except KeyError as e:
                pass
        publisher.TopicPublisherWithTimer.publish_interval = (
            settings['publish_interval'])

    def save_to_dict(self):
        save_dict = {}
        save_dict['texts'] = [x.get_text() for x in self._widget.get_sliders()]
        save_dict['publish_interval'] = (
            publisher.TopicPublisherWithTimer.publish_interval)
        save_dict['settings'] = {}
        for slider in self._widget.get_sliders():
            save_dict['settings'][slider.get_text()] = {}
            range_min, range_max = slider.get_range()
            save_dict['settings'][slider.get_text()]['min'] = range_min
            save_dict['settings'][slider.get_text()]['max'] = range_max
            save_dict['settings'][slider.get_text()]['is_repeat'] = (
                slider.is_repeat())
        return save_dict

    def trigger_configuration(self):
        dialog = config_dialog.ConfigDialog(self)
        dialog.exec_()

    @staticmethod
    def _isfile(parser, arg):
        if os.path.isfile(arg):
            return arg
        else:
            parser.error("Setting file %s does not exist" % arg)

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_ez_publisher plugin')
        group.add_argument('--slider-file',
                           type=lambda x: EzPublisherPlugin._isfile(parser, x),
                           help="YAML setting file")
