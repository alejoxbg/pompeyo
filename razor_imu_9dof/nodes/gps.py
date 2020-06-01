#!/usr/bin/env python
import serial
import time
import string
import pynmea2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class publishGPS(object):

	def __init__(self):
		rospy.loginfo("Initialising GPS publishing")
		self.lastMsg=None
		self.gps_pub=rospy.Publisher('/gps', NavSatFix, queue_size=10)
		rospy.sleep(1)
		self.port="/dev/ttyS0"
		self.ser=serial.Serial(self.port, baudrate=9600, timeout=2)
		self.dataout = pynmea2.NMEAStreamReader()
		rospy.loginfo("initialised")

	def callback(self, data):
		self.lastMsg=data

	def do_work(self):
		self.newdata=self.ser.readline()
		if self.newdata[0:6] == "$GPGGA":
			try:
				#separado = self.newdata.split(",")
				#print("Separado por ',' es:", separado)
				newmsg=pynmea2.parse(self.newdata)
				lat=newmsg.latitude
				lon=newmsg.longitude	
				alt=newmsg.altitude
				gpsmsg=NavSatFix()
				#gpsmsg.header.stamp = rospy.Time.now()
				gpsmsg.header.frame_id = "gps"
				gpsmsg.latitude=lat
				gpsmsg.longitude=lon
				gpsmsg.altitude=alt
				self.gps_pub.publish(gpsmsg)
			except:
				print("Something went wrong")
				#print(separado)


	def run(self):
		r=rospy.Rate(10)
		while not rospy.is_shutdown():
			self.do_work()
			r.sleep()

if __name__=='__main__':
	rospy.init_node('gps')
	obj=publishGPS()
	obj.run()
