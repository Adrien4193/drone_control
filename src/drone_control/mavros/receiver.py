import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

from ..common import Callback, Pose


class Receiver(object):
    
    def __init__(self):
        self._subs = {}
        self._state = State()
        self._pose = Pose()
        
        self._subs['state'] = rospy.Subscriber(
            '/mavros/state',
            State,
            Callback(self, Receiver._on_state_received)
        )
        
        self._subs['estimate'] = rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            Callback(self, Receiver._on_pose_received)
        )
    
    def __del__(self):
        for sub in self._subs.values():
            sub.unregister()
    
    @property
    def connected(self):
        return self._state.connected
    
    @property
    def armed(self):
        return self._state.armed
        
    @property
    def mode(self):
        return self._state.mode
    
    @property
    def pose(self):
        return self._pose
    
    def _on_state_received(self, state):
        self._state = state
    
    def _on_pose_received(self, pose_stamped):
        self._pose = Pose.from_message(pose_stamped)
        
        
        
