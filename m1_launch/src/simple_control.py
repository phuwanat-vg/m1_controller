#!/usr/bin/python3
import rospy
from m1_msgs.msg import M1PtpCmd
from std_msgs.msg import Int16

import time

point=M1PtpCmd()
io = Int16()


def goTo(pub, xin,yin,zin):
  #point.time_from_start=rospy.Duration(0.1)
  point.ptpMode = 1
  point.x = xin
  point.y = yin
  point.z = zin
  pub.publish(point)


def main():
  rospy.init_node("m1_node")
  rate=rospy.Rate(0.2)
  pub=rospy.Publisher('/ptp_cmd', M1PtpCmd, queue_size=1)
  io_pub=rospy.Publisher('/io_cmd', Int16, queue_size=1)

  goTo(pub, 200,0.0,200.0)
  rate.sleep()
  print("success")
  goTo(pub, 215,-10,220.0)
  rate.sleep()
  print("success")
 
  goTo(pub, 230,-10,67.5)
  rate.sleep()
  print("point4")
  
  io = 0
  io_pub.publish(io);
  rate.sleep()
  print("Suction")
  
  print("point3")
  goTo(pub, 230,-10,100)
  rate.sleep()
  print("lifting")
  
  print("point3")
  goTo(pub, 200,100,67.5)
  rate.sleep()
  print("lifting")
  
  io = 1
  io_pub.publish(io);
  rate.sleep()
  print("Spray")
  
  goTo(pub, 200,0.0,200.0)
  rate.sleep()
  print("success")
if __name__ == "__main__":
  main()
