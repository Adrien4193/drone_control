import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


class Commander(object):
	
	def arm(self, state):
		self._call_service('/mavros/cmd/arming', CommandBool, state)
		
	def land(self):
		self._call_service('/mavros/cmd/land', CommandTOL)
		
	def set_mode(self, mode):
		self._call_service('/mavros/set_mode', SetMode, custom_mode=mode)
		
	def takeoff(self):
		self._call_service('/mavros/cmd/takeoff', CommandTOL, altitude=1.0)
	
	def _call_service(self, name, command_type, *args, **kwargs):
		rospy.loginfo(
			'Service called: ' + name
			+ ' args: ' + str(args)
			+ ' kwargs' + str(kwargs)
		)
		try:
			service = rospy.ServiceProxy(name, command_type)
			service(*args, **kwargs)
		except rospy.ServiceException as e:
			rospy.logerr(e)
