from lab_baxter_common.general_toolkit import(
    move_to_posture,
    save_posture
)

import rospy

def main():
    rospy.init_node('saver')
    save_posture('face_podi2')

if __name__ == '__main__':
    main()