#!/usr/bin/env python

import unittest
import geometry_msgs.msg as geo_msgs

import rqt_ez_publisher.ez_publisher_model as ez_model
from rqt_ez_publisher import quaternion_module

PKG='rqt_ez_publisher'

class FunctionTest(unittest.TestCase):

    def test_make_topic_strings(self):
        strings = ez_model.make_topic_strings(geo_msgs.Twist(), '/cmd_vel')
        self.assertEqual(len(strings), 6)
        self.assertTrue('/cmd_vel/linear/x' in strings)
        self.assertTrue('/cmd_vel/linear/y' in strings)
        self.assertTrue('/cmd_vel/linear/z' in strings)
        self.assertTrue('/cmd_vel/angular/x' in strings)
        self.assertTrue('/cmd_vel/angular/y' in strings)
        self.assertTrue('/cmd_vel/angular/z' in strings)

    def test_make_topic_strings_with_header(self):
        strings = ez_model.make_topic_strings(geo_msgs.PointStamped(),
                                              '/cmd_vel')
        self.assertEqual(len(strings), 7)
        self.assertTrue('/cmd_vel/header/seq' in strings)
        self.assertTrue('/cmd_vel/header/stamp/secs' in strings)
        self.assertTrue('/cmd_vel/header/stamp/nsecs' in strings)
        self.assertTrue('/cmd_vel/header/frame_id' in strings)
        self.assertTrue('/cmd_vel/point/x' in strings)
        self.assertTrue('/cmd_vel/point/y' in strings)
        self.assertTrue('/cmd_vel/point/z' in strings)

    def test_flatten(self):
        flattened = ez_model.flatten([0, [[1, 2], 3, 4], [5, 6], [7], 8])
        self.assertEqual(len(flattened), 9)

    def test_find_topic_name_found(self):
        topic, attr, index = ez_model.find_topic_name(
            '/hoge/data', {'/hoge': 'type_a', '/hoga': 'type_b'})
        self.assertEqual(topic, '/hoge')
        self.assertEqual(attr, ['data'])
        self.assertEqual(index, None)

    def test_find_topic_name_found_topic_only(self):
        topic, attr, index = ez_model.find_topic_name(
            '/hoge', {'/hoge': 'type_a', '/hoga': 'type_b'})
        self.assertEqual(topic, '/hoge')
        self.assertEqual(attr, None)
        self.assertEqual(index, None)

    def test_find_topic_name_found_topic_only_nested(self):
        topic, attr, index = ez_model.find_topic_name(
            '/hoge', {'/hoge': 'type_a', '/hoga': 'type_b', '/hoge/nested': 'type_c'})
        self.assertEqual(topic, '/hoge')
        self.assertEqual(attr, None)
        self.assertEqual(index, None)

    def test_find_topic_name_found_topic_only_nested_by_nested(self):
        topic, attr, index = ez_model.find_topic_name(
            '/some/topic/again/data',
            {'/some/topic/again': 'std_msgs/Float32', '/some/topic': 'std_msgs/Float32', '/this/works': 'std_msgs/Float32'})
        self.assertEqual(topic, '/some/topic/again')
#        self.assertEqual(attr, None)
        self.assertEqual(index, None)

    def test_find_topic_name_found_topic_only_nested_by_nested2(self):
        topic, attr, index = ez_model.find_topic_name(
            '/some/topic/again',
            {'/some/topic/again': 'std_msgs/Float32', '/some/topic': 'std_msgs/Float32', '/this/works': 'std_msgs/Float32'})
        self.assertEqual(topic, '/some/topic/again')
        self.assertEqual(attr, None)
        self.assertEqual(index, None)

    def test_find_topic_name_found_with_index(self):
        topic, attr, index = ez_model.find_topic_name(
            '/hoge/data[2]', {'/hoge': 'type_a', '/hoga': 'type_b'})
        self.assertEqual(topic, '/hoge')
        self.assertEqual(attr, ['data'])
        self.assertEqual(index, 2)

    def test_find_topic_name_not_found(self):
        topic, attr, index = ez_model.find_topic_name(
            '/hoge/data', {'/hogi': 'type_a', '/hoga': 'type_b'})
        self.assertEqual(topic, None)
        self.assertEqual(attr, None)
        self.assertEqual(index, None)

    def test_find_topic_name_repeated(self):
        topic, attr, index = ez_model.find_topic_name(
            '/hoge/goal/goal', {'/hoge/goal': 'type_a', '/hoga': 'type_b'})
        self.assertEqual(topic, '/hoge/goal')
        self.assertEqual(attr, ['goal'])
        self.assertEqual(index, None)

    def test_get_value_type(self):
        type, is_array = ez_model.get_value_type(
            'geometry_msgs/Twist', ['linear', 'x'])
        self.assertEqual(type, float)
        self.assertEqual(is_array, False)

    def test_get_value_type_header(self):
        type, is_array = ez_model.get_value_type(
            'geometry_msgs/PointStamped', ['header', 'frame_id'])
        self.assertEqual(type, str)
        self.assertEqual(is_array, False)

    def test_get_value_type_not_found(self):
        type, is_array = ez_model.get_value_type(
            'geometry_msgs/Twist', ['linear'])
        self.assertEqual(type, None)
        self.assertEqual(is_array, False)

    def test_get_value_type_array(self):
        type, is_array = ez_model.get_value_type(
            'geometry_msgs/TwistWithCovariance', ['covariance'])
        self.assertEqual(type, float)
        self.assertEqual(is_array, True)

    def test_get_value_type_non_builtin_array(self):
        type, is_array = ez_model.get_value_type(
            'geometry_msgs/Polygon', ['points[0]', 'x'])
        self.assertEqual(type, float)
        self.assertEqual(is_array, False)

    def test_make_test(self):
        text = ez_model.make_text('/cmd_vel', ['linear', 'x'], None)
        self.assertEqual(text, '/cmd_vel/linear/x')

    def test_make_test_array(self):
        text = ez_model.make_text('/cmd_vel', ['linear', 'x'], 2)
        self.assertEqual(text, '/cmd_vel/linear/x[2]')

    def test_get_value_type_quaternion(self):
        msg_type, is_array = ez_model.get_value_type('geometry_msgs/Pose', ['orientation'])
        self.assertEqual(msg_type, None)
        self.assertEqual(is_array, False)
        msg_type, is_array = ez_model.get_value_type(
            'geometry_msgs/Pose', ['orientation'],
            modules=[quaternion_module.QuaternionModule()])
        self.assertEqual(msg_type, 'geometry_msgs/Quaternion')
        self.assertEqual(is_array, False)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'function_test', FunctionTest)
