import rospy
from geometry_msgs.msg import PoseStamped

from ..common import Callback, Pose


# Slow
def get_trackers_list():
    trackers = []
    topics = rospy.get_published_topics('/vrpn_client_node')

    for topic in topics:
        if 'pose' in topic[0]:
            trackers.append(topic[0].split('/')[-2])
    trackers.sort()

    return trackers


class RigidBody(object):
    
    def __init__(self, tracker_id):
        self._sub = rospy.Subscriber(
            '/vrpn_client_node/'+tracker_id+'/pose',
            PoseStamped,
            Callback(self, RigidBody._on_message_received)
        )
        self._pose = Pose()
        self._timestamp = rospy.get_time()
    
    def __del__(self):
        self._sub.unregister()
    
    @property
    def tracked(self):
        elapsed = rospy.get_time() - self._timestamp
        return elapsed < rospy.get_param('~tracking/timeout')
    
    @property
    def pose(self):
        return self._pose
        
    def _on_message_received(self, pose_stamped):
        self._timestamp = pose_stamped.header.stamp.to_sec()
        self._pose = Pose.from_message(pose_stamped)
    
