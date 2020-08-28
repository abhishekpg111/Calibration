import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransformArray
import numpy as np


from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera, reset

rospy.init_node('hand2eye_calibration_node')

tfBuffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tfBuffer)

tf_fid_broadcaster = tf2_ros.TransformBroadcaster()

# rospy.sleep(1)

world_effector_pub = rospy.Publisher(
    'world_effector', geometry_msgs.msg.Transform, queue_size=1)
camera_object_pub = rospy.Publisher(
    'camera_object', geometry_msgs.msg.Transform, queue_size=1)

listener = tf.TransformListener()

# print tf_listener.lookupTransform('eef_tool_tip', 'robot_base_link', rospy.Time())

# f = rospy.wait_for_message('fiducial_transforms', FiducialTransformArray, 100)


def update_fid(msg):

    if len(msg.transforms) != 1:
        return

    fid_transform = msg.transforms[0].transform

    fid = geometry_msgs.msg.TransformStamped()
    fid.header.frame_id = 'camera_color_optical_frame'
    fid.child_frame_id = 'calibration_object'
    fid.transform.translation.x = fid_transform.translation.x
    fid.transform.translation.y = fid_transform.translation.y
    fid.transform.translation.z = fid_transform.translation.z
    fid.transform.rotation.x = fid_transform.rotation.x
    fid.transform.rotation.y = fid_transform.rotation.y
    fid.transform.rotation.z = fid_transform.rotation.z
    fid.transform.rotation.w = fid_transform.rotation.w

    tf_fid_broadcaster.sendTransform(fid)


rospy.Subscriber('fiducial_transforms', FiducialTransformArray,
                 update_fid, queue_size=10)

print 'system ready...'
print 'press <enter> to register the poses'
print 'type <y> to finish the capturing'

rotations = []
translations = []

while raw_input('') != 'y':
    # /world_effector
    try:

    	listener.waitForTransform("world", "wrist3_Link", rospy.Time(0), rospy.Duration(4.0))
    	listener.waitForTransform("camera_link", "calibration_object", rospy.Time(0), rospy.Duration(4.0))
    	(robot_trans,robot_rot)= listener.lookupTransform("world", "wrist3_Link", rospy.Time(0))
    	(fiducial_trans,fiducial_rot)= listener.lookupTransform("camera_link", "calibration_object", rospy.Time(0))
	rob=geometry_msgs.msg.Transform()
	rob.translation.x=robot_trans[0]
	rob.translation.y=robot_trans[1]
	rob.translation.z=robot_trans[2]
	rob.rotation.x=robot_rot[0]
	rob.rotation.y=robot_rot[1]
	rob.rotation.z=robot_rot[2]
	rob.rotation.w=robot_rot[3]
	fidu=geometry_msgs.msg.Transform()
	fidu.translation.x=fiducial_trans[0]
	fidu.translation.y=fiducial_trans[1]
	fidu.translation.z=fiducial_trans[2]
	fidu.rotation.x=fiducial_rot[0]
	fidu.rotation.y=fiducial_rot[1]
	fidu.rotation.z=fiducial_rot[2]
	fidu.rotation.w=fiducial_rot[3]
	print "robot\n",rob
	print "fiducial\n",fidu
        world_effector_pub.publish(rob)
        camera_object_pub.publish(fidu)
        print '<saved state>'
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("nao consegui ter as trans")

    # /camera_object
    # print robot.transform
    # print fiducial.transform

    # T1 = robot.transform.translation
    # T2 = fiducial.transform.translation
    # T1 = tf.transformations.translation_matrix([T1.x, T1.y, T1.z])
    # T2 = tf.transformations.translation_matrix([T2.x, T2.y, T2.z])

    # R1 = robot.transform.rotation
    # R2 = fiducial.transform.rotation
    # R1 = tf.transformations.quaternion_matrix([R1.x, R1.y, R1.z, R1.w])
    # R2 = tf.transformations.quaternion_matrix([R2.x, R2.y, R2.z, R2.w])
    
    # T = T1.dot(R1).dot(T2).dot(R2)

    # # print T

    # Rot = tf.transformations.quaternion_from_matrix(T)
    # Tr = tf.transformations.translation_from_matrix(T)
    # rotations.append(Rot)
    # translations.append(Tr)

    # print 'xyz = ', Tr
    # print 'rpy = ', Rot

print '======================'
print 'Printing Final Results'
print '======================'

# rotations = np.array(rotations).mean(axis=0)
# translations = np.array(translations).mean(axis=0)

# print 'xyz = ', translations
# print 'rpy = ', rotations

calibratorService = rospy.ServiceProxy(
    'compute_effector_camera', compute_effector_camera)

result = calibratorService()
x=result.effector_camera.rotation.x
y=result.effector_camera.rotation.y
z=result.effector_camera.rotation.z
w=result.effector_camera.rotation.w

print result
print "x=",x
print "y=",y
print "z=",z
print "w=",w

quaternion = (x,y,z,w)

#convertion from quaternions(xyzw) to euler angles(rpy) which in ROS is in the YZX format
euler = tf.transformations.euler_from_quaternion(quaternion)
roll = euler[0]
pitch = euler[1]
yaw = euler[2]

print "roll=",roll
print "pitch=",pitch
print "yaw=",yaw

print roll,pitch,yaw
resetService = rospy.ServiceProxy('reset', reset)
resetService()
