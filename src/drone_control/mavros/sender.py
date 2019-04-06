import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from mavros_msgs.msg import Thrust


class Sender(object):

    def __init__(self):
        self._pubs = {}
        
        self._pubs['position'] = rospy.Publisher(
            '/mavros/setpoint_position/local',
			PoseStamped, queue_size=10
        )
        
        self._pubs['attitude'] = rospy.Publisher(
            '/mavros/setpoint_attitude/attitude',
			PoseStamped, queue_size=10
        )
        
        self._pubs['velocity'] = rospy.Publisher(
            '/mavros/setpoint_attitude/velocity',
			PoseStamped, queue_size=10
        )
        
        self._pubs['thrust'] = rospy.Publisher(
            '/mavros/setpoint_attitude/thrust',
			Thrust, queue_size=10
        )

        self._pubs['mocap'] = rospy.Publisher(
            '/mavros/mocap/pose',
            PoseStamped, queue_size=10
        )
    
    def __del__(self):
        for pub in self._pubs.values():
            pass#pub.unregister()
    
    def send_attitude(self, attitude):
        self._pubs['attitude'].publish(attitude.get_message())
        self._pubs['thrust'].publish(Thrust(thrust=attitude.thrust))
    
    def send_velocity(self, attitude):
        self._pubs['velocity'].publish(attitude.get_message())
        self._pubs['thrust'].publish(Thrust(thrust=attitude.thrust))
    
    def send_position(self, pose):
        self._pubs['position'].publish(pose.get_message())
    
    def send_mocap(self, pose):
        self._pubs['mocap'].publish(pose.get_message())
        
        
        
