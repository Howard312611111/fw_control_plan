#!/usr/bin/env python3

import csv
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from fw_control.msg import EstimateOutput

class Node():

	def __init__(self, keyboard = False):

		self.pose_esti_pub = rospy.Publisher("/uav0/StateEstimation/PoseEstimation", Vector3, queue_size=10)
		self.vel_esti_pub = rospy.Publisher("/uav0/StateEstimation/VelEstimation", Vector3, queue_size=10)
		self.pose_truth_pub = rospy.Publisher("/uav0/StateEstimation/PoseTruth", Vector3, queue_size=10)
		self.vel_truth_pub = rospy.Publisher("/uav0/StateEstimation/VelTruth", Vector3, queue_size=10)

		self.output_sub = rospy.Subscriber("/uav0/estimation/ukf/output_data", EstimateOutput, self.output_callback)
		self.groundTruth_sub = rospy.Subscriber("/uav0/estimation/ukf/groundtruth", EstimateOutput, self.groundTruth_callback)

	def output_callback(self, msg):

		output_px = msg.target_pose.x
		output_py = msg.target_pose.y
		output_pz = msg.target_pose.z
		output_vx = msg.target_vel.x
		output_vy = msg.target_vel.y
		output_vz = msg.target_vel.z

		output_pose = Vector3(x = output_px, y = output_py, z = output_pz)
		output_vel = Vector3(x = output_vx, y = output_vy, z = output_vz)
		self.pose_esti_pub.publish(output_pose)
		self.vel_esti_pub.publish(output_vel)

		print("Estimation_Output: ", output_pose)

		filename = "/home/leo/plan/src/fw_control/data/EmEstimation/StaticPrius_PoseEstimation.csv"
		with open(filename, "a", encoding='UTF8', newline='') as f:

			row = [output_px, output_py, output_pz]
			writer = csv.writer(f)
			writer.writerow(row)

		filename = "/home/leo/plan/src/fw_control/data/EmEstimation/StaticPrius_VelEstimation.csv"
		with open(filename, "a", encoding='UTF8', newline='') as f:

			row = [output_vx, output_vy, output_vz]
			writer = csv.writer(f)
			writer.writerow(row)

	def groundTruth_callback(self, msg):

		output_px = msg.target_pose.x
		output_py = msg.target_pose.y
		output_pz = msg.target_pose.z
		output_vx = msg.target_vel.x
		output_vy = msg.target_vel.y
		output_vz = msg.target_vel.z

		output_pose = Vector3(x = output_px, y = output_py, z = output_pz)
		output_vel = Vector3(x = output_vx, y = output_vy, z = output_vz)
		self.pose_truth_pub.publish(output_pose)
		self.vel_truth_pub.publish(output_vel)

		print("GroundTruth_Output: ", output_pose)

		filename = "/home/leo/plan/src/fw_control/data/EmEstimation/StaticPrius_PoseTruth.csv"
		with open(filename, "a", encoding='UTF8', newline='') as f:

			row = [output_px, output_py, output_pz]
			writer = csv.writer(f)
			writer.writerow(row)

		filename = "/home/leo/plan/src/fw_control/data/EmEstimation/StaticPrius_VelTruth.csv"
		with open(filename, "a", encoding='UTF8', newline='') as f:

			row = [output_vx, output_vy, output_vz]
			writer = csv.writer(f)
			writer.writerow(row)


if __name__ == '__main__':

	rospy.init_node('DataC', anonymous=True)

	rate = rospy.Rate(100)

	Collector = Node()

	filename = "/home/leo/plan/src/fw_control/data/EmEstimation/StaticPrius_PoseEstimation.csv"
	f = open(filename, "w+")
	f.close()

	filename = "/home/leo/plan/src/fw_control/data/EmEstimation/StaticPrius_VelEstimation.csv"
	f = open(filename, "w+")
	f.close()

	filename = "/home/leo/plan/src/fw_control/data/EmEstimation/StaticPrius_PoseTruth.csv"
	f = open(filename, "w+")
	f.close()

	filename = "/home/leo/plan/src/fw_control/data/EmEstimation/StaticPrius_VelTruth.csv"
	f = open(filename, "w+")
	f.close()

	while not rospy.is_shutdown():

		rate.sleep()