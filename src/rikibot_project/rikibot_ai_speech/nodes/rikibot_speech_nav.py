#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: kevin <304050118@qq.com>

import rospy
import os
import sys
import time
from threading import Timer
from audio_common_msgs.msg import AudioData
from rikibot_ai_speech import RikibotAISpeech
from geometry_msgs.msg import Pose, Point, Quaternion
from rikibot_nav import RikibotNav

class RikibotSpeechNav(object):
    def __init__(self):
        self.register_code = os.environ['RIKI_SERIAL']
        self.sub_audio = rospy.Subscriber("speech_audio", AudioData, self.audio_cb)
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.dir_path = self.dir_path.replace('rikibot_ai_speech/nodes', 'rikibot_ai_speech/')
        self.dir_path += 'voice/'
        self.order = [u'厨房', u'客厅',u'阳台']

        self.ai = RikibotAISpeech(0, self.dir_path, register_code=self.register_code)
	self.audio_text = None
        self.is_canceling = False
        self.start_nav = False	
	self.locations = dict()
        self.locations[self.order[0]]   = Pose(Point(1.025, 0.995, 0.000), Quaternion(0.000, 0.000, 0.009, 1.000))
        self.locations[self.order[1]]   = Pose(Point(4.016, 0.508, 0.000), Quaternion(0.000, 0.000, 1.000, -0.019))
        self.locations[self.order[2]]   = Pose(Point(1.247, -0.372, 0.000), Quaternion(0.000, 0.000, 1.000, -0.031))

        self.nav = RikibotNav()

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.start_nav is True:
                state =self.nav.NavState()
                if state == 1:
                    self.start_nav = False
            r.sleep()

    def audio_cb(self, msg):
        if self.is_canceling is True:
            return

        self.audio_text = self.ai.audio_to_text(msg.data)	
        if self.audio_text is not None:
            rospy.loginfo("You Said: %s", self.audio_text.encode('utf-8'))
            self.is_canceling = True
            if self.audio_text.find(self.order[0]) != -1:
                rospy.loginfo("Start Navigation: %s", self.order[0].encode('utf-8'))
                self.nav.SendGoal(self.locations[self.order[0]])
                #self.ai.text_to_audio(u"好的，小锐现在去" + self.order[0])
                #self.ai.audio_play()
                self.start_nav = True
            if self.audio_text.find(self.order[1]) != -1:
                rospy.loginfo("Start Navigation: %s", self.order[1].encode('utf-8'))
                self.nav.SendGoal(self.locations[self.order[1]])
                #self.ai.text_to_audio(u"好的，小锐现在去" + self.order[1])
                #self.ai.audio_play()
                self.start_nav = True
            if self.audio_text.find(self.order[2]) != -1:
                rospy.loginfo("Start Navigation: %s", self.order[2].encode('utf-8'))
                self.nav.SendGoal(self.locations[self.order[2]])
                #self.ai.text_to_audio(u"好的，小锐现在去" + self.order[2])
                #self.ai.audio_play()
                self.start_nav = True


            self.is_canceling = False


if __name__ == '__main__':
    rospy.init_node("Rikibot_Speech_Nav_Node")
    rikibotai = RikibotSpeechNav()
    rospy.spin()
