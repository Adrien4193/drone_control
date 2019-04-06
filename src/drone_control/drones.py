from mavros import Sender, Receiver, Commander
from mocap import RigidBody
from common import Attitude, Pose, Timer


PX4_FLIGHT_MODES = [
    'MANUAL',
    'ACRO',
    'ALTCTL',
    'POSCTL',
    'OFFBOARD',
    'STABILIZED',
    'RATTITUDE',
    'AUTO.MISSION',
    'AUTO.LOITER',
    'AUTO.RTL',
    'AUTO.LAND',
    'AUTO.RTGS',
    'AUTO.READY',
    'AUTO.TAKEOFF'
]


class BasicDrone(object):

    def __init__(self):
        self.__sender = Sender()
        self.__receiver = Receiver()
        self.__commander = Commander()
    
    @property
    def connected(self):
        return self.__receiver.connected

    @property
    def armed(self):
        return self.__receiver.armed
    
    @armed.setter
    def armed(self, val):
        self.arm(val)

    @property
    def mode(self):
        return self.__receiver.mode
    
    @mode.setter
    def mode(self, val):
        self.set_mode(val)

    @property
    def estimate(self):
        return self.__receiver.pose
    
    def arm(self, state):
        self.__commander.arm(state)
    
    def land(self):
        self.__commander.land()
    
    def set_mode(self, mode):
        self.__commander.set_mode(mode)
    
    def takeoff(self):
        self.__commander.takeoff()
    
    def send_attitude(self, attitude):
        self.__sender.send_attitude(attitude)
    
    def send_position(self, pose):
        self.__sender.send_position(pose)
    
    def send_velocity(self, attitude):
        self.__sender.send_velocity(attitude)
    
    def send_mocap(self, pose):
        self.__sender.send_mocap(pose)


class TrackedDrone(BasicDrone):

    def __init__(self, tracker_id):
        super(TrackedDrone, self).__init__()

        self.__body = RigidBody(tracker_id)

    @property
    def tracked(self):
        return self.__body.tracked

    @property
    def pose(self):
        return self.__body.pose
    
    def send_mocap_to_fcu(self):
        self.send_mocap(self.pose)


class ControlledDrone(TrackedDrone):

    def __init__(self, tracker_id, control_rate=60):
        super(ControlledDrone, self).__init__(tracker_id)
    
        self.__attitude = Attitude()
        self.__pose = Pose()

        self.__timers = []
        self.__setup_timers(control_rate)
        
    def __del__(self):
        for timer in self.__timers:
            timer.stop()
    
    @property
    def attitude(self):
        return self.__attitude
    
    @attitude.setter
    def attitude(self, val):
        self.__attitude = val
    
    @property
    def pose(self):
        return self.__pose
    
    @pose.setter
    def pose(self, val):
        self.__pose = val
    
    def __setup_timers(self, control_rate):
        self.__timers.append(Timer(control_rate, self.__send_attitude))

        for timer in self.__timers:
            timer.start()

    def __send_attitude(self):
        self.send_attitude(self.__attitude)


