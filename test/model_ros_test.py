#!/usr/bin/env python

import rostest
import unittest
import rospy
import geometry_msgs.msg as geo_msgs
import sensor_msgs.msg as sen_msgs

import rqt_ez_publisher.ez_publisher_model as ez_model

PKG='rqt_ez_publisher'

def wait_topics():
    _, _, topic_types = rospy.get_master().getTopicTypes()
    topic_dict = dict(topic_types)
    while '/joint_states' not in topic_dict or '/polygon' not in topic_dict:
        rospy.sleep(0.1)


# check /joint_states and /polygon
class RosFunctionTest(unittest.TestCase):

    def callback(self, msg):
        pass

    def setUp(self):
        wait_topics()

    def test_make_topic_strings_with_array_non_builtin(self):
        strings = ez_model.make_topic_strings(geo_msgs.Polygon(), '/polygon')
        self.assertEqual(len(strings), 1)
        self.assertEqual(strings[0][0], '/polygon/points[0]/x')
        self.assertEqual(strings[0][1], '/polygon/points[0]/y')
        self.assertEqual(strings[0][2], '/polygon/points[0]/z')

    def test_make_topic_strings_with_array_builtin(self):
        strings = ez_model.make_topic_strings(sen_msgs.JointState(),
                                              '/joint_states')
        self.assertEqual(len(strings), 5)
        self.assertEqual(strings[1], '/joint_states/name[0]')
        self.assertEqual(strings[2], '/joint_states/position[0]')



class TopicPublisherTest(unittest.TestCase):
    
    def callback(self, msg):
        self._msg = msg

    def setUp(self):
        self.publiser = ez_model.TopicPublisher('/topic', geo_msgs.Vector3)
        self.subscriber = rospy.Subscriber('/topic', geo_msgs.Vector3, self.callback)
        while self.subscriber.get_num_connections() > 0:
            rospy.sleep(0.1)
        self._msg = None

    def test_get_topic_name(self):
        self.assertEqual(self.publisher.get_topic_name(), '/topic')
        
    def test_publish(self):
        msg = self.publisher.get_message()
        msg.x = 1.0
        msg.y = 2.0
        msg.z = 3.0
        self.publisher.publish()
        while not self._msg:
            rospy.sleep(0.1)
        self.assertEqual(self._msg.x, 1.0)
        self.assertEqual(self._msg.y, 2.0)
        self.assertEqual(self._msg.z, 3.0)

        
class ModelTest(unittest.TestCase):

    def setUp(self):
        wait_topics()
        self.model = ez_model.EasyPublisherModel()

    def test_get_publisher_not_exists(self):
        self.assertEqual(self.model.get_publisher('not_exists'), None)

    def test_expand_attribute_polygon(self):
        strings = self.model.expand_attribute('/polygon/points', None)
        self.assertEqual(len(strings), 3)
        self.assertEqual(strings[0], '/polygon/points[0]/x')
        self.assertEqual(strings[1], '/polygon/points[0]/y')
        self.assertEqual(strings[2], '/polygon/points[0]/z')

    def test_expand_attribute_with_index(self):
        strings = self.model.expand_attribute('/polygon/points', 1)
        self.assertEqual(len(strings), 3)
        self.assertEqual(strings[0], '/polygon/points[1]/x')
        self.assertEqual(strings[1], '/polygon/points[1]/y')
        self.assertEqual(strings[2], '/polygon/points[1]/z')

    def test_expand_attribute_polygon(self):
        strings = self.model.expand_attribute('/polygon')
        self.assertEqual(len(strings), 3)
        self.assertEqual(strings[0], '/polygon/points[0]/x')
        self.assertEqual(strings[1], '/polygon/points[0]/y')
        self.assertEqual(strings[2], '/polygon/points[0]/z')

    def test_expand_attribute_array(self):
        strings = self.model.expand_attribute('/joint_states/position', 2)
        self.assertEqual(len(strings), 1)
        self.assertEqual(strings[0], '/joint_states/position[2]')

    def test_expand_attribute_joint_states_header(self):
        strings = self.model.expand_attribute('/joint_states/header/stamp/secs')
        self.assertEqual(len(strings), 0)

    def test_expand_attribute_joint_states(self):
        strings = self.model.expand_attribute('/joint_states')
        self.assertEqual(len(strings), 8)
        self.assertEqual(strings[0], '/joint_states/header/seq')
        self.assertEqual(strings[1], '/joint_states/header/stamp/secs')
        self.assertEqual(strings[2], '/joint_states/header/stamp/nsecs')
        self.assertEqual(strings[3], '/joint_states/header/frame_id')
        self.assertEqual(strings[4], '/joint_states/name[0]')
        self.assertEqual(strings[5], '/joint_states/position[0]')
        self.assertEqual(strings[6], '/joint_states/velocity[0]')
        self.assertEqual(strings[7], '/joint_states/effort[0]')


if __name__ == '__main__':
    import rostest
    rospy.init_node('ez_publisher_model_test')
    rostest.rosrun(PKG, 'ros_function_test', RosFunctionTest)
    rostest.rosrun(PKG, 'model_test', ModelTest)
