#! /usr/bin/python

from multibot_relay import (
    MultiBotInterface
)
import rospy

def main():
    rospy.init_node('first-part')
    mbot_relay = MultiBotInterface()
    mbot_relay.send_signal('move-to-hidden')
    # mbot_relay.wait_for_signal('done', _id='podi') #wait for done
    # rospy.sleep(1)
    # mbot_relay.send_signal('move-to-hidden')

if __name__ == '__main__':
    main()

