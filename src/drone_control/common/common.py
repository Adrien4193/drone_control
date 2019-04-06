import threading
import weakref
import numpy

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion, PoseStamped


class Pose(object):

    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    
    @staticmethod
    def from_message(pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
		
        q = poseStamped.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
		
        return Pose(x, y, z, roll, pitch, yaw)
    
    def to_array(self):
        return numpy.array(
            self.x, self.y, self.z,
            self.roll, self.pitch, self.yaw
        )
    
    def get_message(self):
        pose = PoseStamped()

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z

        pose.pose.orientation = Quaternion(
            *quaternion_from_euler(self.roll, self.pitch, self.yaw)
        )

        return pose
		

class Attitude(object):
    
    def __init__(self, roll=0, pitch=0, yaw=0, thrust=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.thrust = thrust
    
    @staticmethod
    def from_message(pose):
        q = poseStamped.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        return Pose(x, y, z, roll, pitch, yaw)
    
    def to_array(self):
        return numpy.array(
            self.roll, self.pitch, self.yaw, self.thrust
        )
    
    def get_message(self):
        pose = PoseStamped()

        pose.pose.orientation = Quaternion(
            *quaternion_from_euler(self.roll, self.pitch, self.yaw)
        )

        return pose

class Callback(object):
    
    def __init__(self, obj, method):
        self._obj = weakref.ref(obj)
        self._method = method
    
    def __call__(self, *args, **kwargs):
        obj = self._obj()
        if obj is not None:
            return self._method(obj, *args, **kwargs)


class Timer(object):
    
    def __init__(self, rate, function):
        self._rate = rospy.Rate(rate)
        self._function = function
        self._running = False
    
    def __del__(self):
        self.stop()
    
    @property
    def running(self):
        return self._running
        
    def start(self):
        self._running = True
        threading.Thread(target=self._run).start()
    
    def stop(self):
        self._running = False
    
    def _run(self):
        while self.running and not rospy.is_shutdown():
            try:
                self._function()
            except Exception as e:
                rospy.logdebug(e)
                break
            self._rate.sleep()
        self._function = None
