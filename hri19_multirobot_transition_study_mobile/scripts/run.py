from geometry_msgs.msg import(
    Pose
)

from podi_2dnav import(
    Navigation
)
import rospy
import rospkg
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
from multibot_relay import MultiBotInterface
from sensor_msgs.msg import Joy
def main():


    rospy.init_node('multi_podi')

    relay_interface = MultiBotInterface()
    print("relay interface started")
    nav = Navigation()
    print('navigaiton started')

    rsp = rospkg.RosPack()
    self._package_path = rsp.get_path('hri19_multirobot_transition_study_mobile')

    # rospy.sleep(1)
    sound_client = SoundClient(blocking=True)
    print('sound client started')
    def play_beep():
        
        sound_client.playWave(os.path.join(self._package_path,"res/audio","beep.wav"))

    def joy_cb(msg):
        if msg.buttons[1]:
            play_beep()
        if msg.buttons[3]: #yellow
            relay_interface.send_signal('done')


    joy_callback = rospy.Subscriber('joy', Joy, joy_cb)
    play_beep()
    print('initialization complete')

    while True:

        signal = relay_interface.wait_for_robot('baxter')
        print(signal)
        if signal == 'move-next-to-robot':
            #define the position
            # next_to_robot = Pose()
            # next_to_robot.position.x = 1.14547458281 
            # next_to_robot.position.y = -1.45922388574 
            # next_to_robot.orientation.w = 0.394609068261 
            # next_to_robot.orientation.z = 0.918849107986   

            next_to_robot = Pose()
            next_to_robot.position.x = 0.516373080689  
            next_to_robot.position.y = -1.02683171536 
            next_to_robot.orientation.w = 0.94768157177 
            next_to_robot.orientation.z = 0.319217227805   

            nav.goto_goal(next_to_robot)
            relay_interface.send_signal('done')

        elif signal == 'beep':
            play_beep()
            relay_interface.send_signal('done')

        elif signal == 'move-to-hidden':

            hide_pose = Pose()
            hide_pose.position.x = -3.74030249205     
            hide_pose.position.y =  0.41707707562 
            hide_pose.orientation.w =   0.778387509459   
            hide_pose.orientation.z =   -0.627784107093  

            nav.goto_goal(hide_pose)
            relay_interface.send_signal('done')

        elif signal == 'move-to-dest':

            dest_pose = Pose()
            dest_pose.position.x =  -14.9592357417     
            dest_pose.position.y =  -0.912321961264  
            dest_pose.orientation.w =   0.714381855724      
            dest_pose.orientation.z =   -0.699756074795  

            nav.goto_goal(dest_pose)
            play_beep()
            relay_interface.send_signal('done')


if __name__ == '__main__':
    main()