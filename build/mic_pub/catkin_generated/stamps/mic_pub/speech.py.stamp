import rospy
from std_msgs.msg import String
import subprocess
import os

rospy.init_node('speech')

def say_text(text):
    print(f"speaking {text}")
    com = 'echo '+text+'|festival --tts --language russian'
    print(com)
    subprocess.call(com, shell=True)
    print("end speaking")

def play_sound(name):
    com = 'aplay ' + str(os.getcwd()) + '/sounds/' + name
    print(com)
    subprocess.call(com, shell=True)


def text_callback(data):
    say_text(data.data)

def sound_callback(data):
    play_sound(data.data)


text_sub = rospy.Subscriber('speech', String, text_callback)
sound_sub = rospy.Subscriber('sound', String, sound_callback)

rospy.spin()


