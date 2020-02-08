#!/usr/bin/env/python

import unittest
from au_core_utils.loader_util import *


class TestLoaderUtil(unittest.TestCase):
    def test_load_available_topic(self):
        self.assertEqual(load_topic("/topic/test"), "/test")

    def test_load_unavailable_topic(self):
        self.assertRaises(Exception, load_topic, '/test_topic')

    def test_available_frame(self):
        self.assertEqual(load_frame("test"), "test_frame")

    def test_unavailable_frame(self):
        self.assertRaises(Exception, load_frame, 'test_again')


if __name__ == '__main__':
    import rosunit
    rosunit.rosrun('au_core', 'test_loader_util', TestLoaderUtil)