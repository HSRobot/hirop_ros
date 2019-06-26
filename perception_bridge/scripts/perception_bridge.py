#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import hirop_perception
import rospy
import ecto, ecto_pcl, ecto_ros
import sys
import time
import os
from hirop_msgs.srv import *


class PerceptionSever():

	count = 0

	def __init__(self):

		self.regionFilter = hirop_perception.RegionFilters("regionFilter", maxZ=1)
		self.pclFunsion = hirop_perception.PclFusion("pclFusion")
		self.rosSource = hirop_perception.PointCloudRos('source', topic_name="/camera/depth/points", world_frame='base_link')
		self.pclViewer = ecto_pcl.CloudViewer("viewer", window_name="PCD Viewer")
		self.saver = ecto_pcl.PCDWriter("saver", filename_format="/home/eima/test_%04u.pcd", binary=True)
		self.loader = ecto_pcl.PCDReader("Reader", filename="/home/eima/test_0000.pcd")
		self.publisher = hirop_perception.PointCloudPublish("publisher", frame_id="base_link")

		self.lookPlasm = ecto.Plasm()
		self.lookPlasm.connect(self.rosSource["output"] >> self.pclFunsion["input"],\
			self.rosSource["T"] >>self.pclFunsion["T"], self.rosSource["R"] >>self.pclFunsion["R"], \
			self.pclFunsion["output"] >> self.pclViewer["input"], self.pclFunsion["output"] >> self.publisher["input"])

		self.savePlasm = ecto.Plasm()
		self.savePlasm.connect(self.rosSource["output"] >> self.pclFunsion["input"],\
			self.rosSource["T"] >>self.pclFunsion["T"], self.rosSource["R"] >>self.pclFunsion["R"], \
			self.pclFunsion["output"] >> self.saver["input"])

		self.loadPlasm = ecto.Plasm()
		self.loadPlasm.connect(self.loader["output"] >> self.pclFunsion["input"], \
			self.pclFunsion["output"] >> self.pclViewer["input"])

		self.testPlasm = ecto.Plasm()
		self.testPlasm.connect(self.rosSource["output"] >> self.pclFunsion["input"], self.pclFunsion["output"] >>  self.regionFilter["input"],\
					self.pclFunsion["output"] >>  self.pclViewer["input"])


	def start(self):
		self.lookService = rospy.Service('look', Look, self.handle_look)
		self.savePclService = rospy.Service('save_pcl', SavePCL, self.handle_save)
		self.loadPclService = rospy.Service('load_pcl', LoadPCL, self.handle_load)
		self.cleanPclService = rospy.Service('clean_pcl', CleanPCL, self.handle_clean)

	def handle_look(self, req):
		print("in look service call back")
		self.sched = ecto.Scheduler(self.testPlasm)
		self.sched.execute(niter=1)
		return LookResponse(0)

	def handle_save(self, req):
		print("saving pcl to pcd file")
		print("file name = ", req.fileName)
		#self.saver.params.filename_format = req.fileName

		self.pclFunsion.params.save = True
		self.sched = ecto.Scheduler(self.savePlasm)
		self.sched.execute(niter=1)
		self.pclFunsion.params.save = False
		return SavePCLResponse(0)

	def handle_load(self, req):
		print "file name = ", req.fileName
		self.sched = ecto.Scheduler(self.loadPlasm)
		self.sched.execute(niter=1)
		return LoadPCLResponse(0)

	def handle_clean(self, req):
		print "cleannig point cloude"
		self.pclFunsion.params.clean = True
		self.sched = ecto.Scheduler(self.lookPlasm)
		self.sched.execute(niter=1)
		self.pclFunsion.params.clean = False
		return CleanPCLResponse(0)


if __name__=="__main__":
	ecto_ros.init(sys.argv, "ros_test")
	rospy.init_node('ros_test')
	p = PerceptionSever()
	p.start()
	rospy.spin()
