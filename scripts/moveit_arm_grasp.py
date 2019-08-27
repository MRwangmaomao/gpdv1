import rospy,sys
import moveit_commander 
import GraspConfigList.msg

# grasps = []
 
# def callback(msg):
#     global grasps
#     grasps = msg.grasps
#     print("I get info-----------------------------")
  
 
def MoveItDemo():
    rospy.init_node('gpd_moveit_demo', anonymous=True)
    # Subscribe to the ROS topic that contains the grasps.
    # sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)

    arm = moveit_commander.MoveGroupCommander('manipulator')
    arm.set_goal_joint_tolerance(0.01)
    # Wait for grasps to arrive.
    rate = rospy.Rate(1)
 
    # while not rospy.is_shutdown():    
    #     if len(grasps) > 0: 
    #         rospy.loginfo('Received %d grasps.', len(grasps))
    #         break
    #     rate.sleep()
    joint_positions = [-0.0867,-0.7,0.02,0.082,-1.273,-0.003]
    arm.set_joint_value_target(joint_positions) 
    arm.go()
    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:  
        MoveItDemo()
    except rospy.ROSInternalException:
        pass