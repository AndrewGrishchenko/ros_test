#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray


help_dict = {
    'ноль': 0,
    'один': 1,
    'два': 2,
    'три': 3,
    'четыре': 4,
    'пять': 5,
    'шесть': 6,
    'семь': 7,
    'восемь': 8,
    'девять': 9,
    'десять': 10,
    'одиннадцать': 11,
    'двенадцать': 12,
    'тринадцать': 13,
    'четырнадцать': 14,
    'пятнадцать': 15,
    'шестнадцать': 16,
    'семнадцать': 17,
    'восемьнадцать': 18,
    'девятнадцать': 19,
    'двадцать': 20,
    'тридцать': 30,
    'сорок': 40,
    'пятьдесят': 50,
    'шестьдесят': 60,
    'семьдесят': 70,
    'восемьдесят': 80,
    'девяносто': 90,
    'сто': 100,
    'двести': 200,
    'триста': 300,
    'четыреста': 400,
    'пятьсот': 500,
    'шестьсот': 600,
    'семьсот': 700,
    'восемьсот': 800,
    'девятьсот': 900,
}

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    speech = rospy.Publisher('speech', String, queue_size=10)
    servo = rospy.Publisher('servo', Int32MultiArray, queue_size=10)
    s = data.data
    send = Int32MultiArray()
    #print(len(s.split()) - 1 - s.split().index("скажи"), s.split()[s.split().index("скажи")+1:])
    if "протокол защиты" in s or "протокол защита" in s:
        #to_say = "Здравствуйте уважаемые члены жюри и гости! Я - робот 'подручный'. Далее защиту продолжит мой программист Черемисова Мария"
        to_say = "Приятно познакомиться, Игорь Вячеславович! Я - робот подручный. Очень надеюсь, что я скоро встану на ноги"
        speech.publish(to_say)
    if "скажи" in s and len(s.split()) - 1 - s.split().index("скажи") > 0:
        to_say = ' '.join(s.split()[s.split().index("скажи")+1:])
        #print(to_say, type(to_say))
        speech.publish(to_say)
    if "привет" in s:
        speech.publish('я робот')
        send.data = [90, 90]
        servo.publish(send)
    if "пока" in s:
        speech.publish('адьос амигос')
        send.data = [0, 0]
        servo.publish(send)
    if "иди" in s or "пошел" in s:
        speech.publish('я слишком стар для такого')
        #go() 
    num = -1
    for i in s.split():
        if i in help_dict:
            num += help_dict[i]
    #rospy.loginfo(rospy.get_caller_id() + str(num) + " " + str(old_num))
    if (num > 180):
        num = -1

    #pub = rospy.Publisher('servo', Int32, queue_size=10)
    if num != -1:
        send.data = [243, num]
        servo.publish(send)
        send.data = [244, num]
        servo.publish(send)

def go():
    servo = rospy.Publisher('servo', Int32MultiArray, queue_size=10) 
    #music
    send = Int32MultiArray()
    for i in range(10):
        send.data = [45, 45]
        servo.publish(send)
        time.sleep(1)
        send.data = [115, 115]
        servo.publish(send)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('reco', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
