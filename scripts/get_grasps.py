import rospy
from gpd.msg import GraspConfigList
import tf
import tf.transformations as tft
import numpy as np
import roslib
# import moveit_commander  


# roslib.load_manifest('learning_tf')

rows = np.array([[0, 0, 0, 1]]) 
grasps = []
trans = None
rot = None

def callback(msg):
    global grasps
    global rot
    global trans
    grasps = msg.grasps
    max_score = 0
    max_score_index = None
    if len(grasps) != 0:
        for grasp_it in grasps:
            print(grasp_it.score.data)
            if grasp_it.score.data > max_score:
                max_score = grasp_it.score.data
                max_score_index = grasp_it

        print("max score:",max_score_index.score.data)
        R = [[max_score_index.approach.x,max_score_index.approach.y,max_score_index.approach.z],[max_score_index.binormal.x,max_score_index.binormal.y,max_score_index.binormal.z],
        [max_score_index.axis.x,max_score_index.axis.y,max_score_index.axis.z]]
        p_grasp = [max_score_index.bottom.x, max_score_index.bottom.y, max_score_index.bottom.z]
        p_grasp = np.array(p_grasp) 
        R = np.array(R) 
        # q_grasp = tft.quaternion_from_matrix(R)
        R = np.c_[R,p_grasp] 
        R_T = np.r_[R, rows] 
        print(R_T) 
        rot = tft.quaternion_matrix(rot) 
        trans = np.array(trans) 
        trans = np.r_[trans.T, [1]]
        print(trans)

        World_R_T = np.c_[rot[:,0:3],trans] 
        print(World_R_T) 
        
        grasp_world_R_T = np.matmul(World_R_T, R_T) 
        print("##################################")
        print(grasp_world_R_T)
    else:
        print("#################no#################")    

    # Create a ROS node.
rospy.init_node('get_grasps')

# Subscribe to the ROS topic that contains the grasps.
sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)
 

listener = tf.TransformListener()
# Wait for grasps to arrive.
rate = rospy.Rate(1)
 
# arm = moveit_commander.MoveGroupCommander('ur5_arm')
# arm.set_goal_joint_tolerance(0.01) 


while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/simple_gripper_gripper_base_link', '/camera_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    if len(grasps) > 0:
        rospy.loginfo('Received %d grasps.', len(grasps)) 
        break
    rate.sleep()
