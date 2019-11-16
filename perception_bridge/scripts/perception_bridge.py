#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import hirop_perception
import rospy
import ecto, ecto_pcl, ecto_ros
import sys
import time
import os
import signal
from threading import Lock
from hirop_msgs.srv import *


lock = Lock()

class PerceptionSever():

	def __init__(self):
		self.regionFilter = hirop_perception.RegionFilters("regionFilter", maxZ=2)
		self.pclFunsion = hirop_perception.PclFusion("pclFusion")
		self.rosSource = hirop_perception.PointCloudRos('source', topic_name="/kinect2/qhd/points", world_frame='base_link')
		self.pclViewer = ecto_pcl.CloudViewer("viewer", window_name="PCD Viewer")
		self.saver = ecto_pcl.PCDWriter("saver", filename_format="/home/eima/test_%04u.pcd", binary=True)
		self.loader = ecto_pcl.PCDReader("Reader", filename="/home/eima/test_0000.pcd")
		self.publisher = hirop_perception.PointCloudPublish("publisher", topic_name="/filter_points", frame_id="base_link")
		self.objectFilter = hirop_perception.ObjectFilter("objectFilter", frame_id="base_link", hight=0.20, width=0.13, length=0.15)
		self.voxelFilter = hirop_perception.VoxelFilter("voxelFilter")

		self.lookPlasm = ecto.Plasm()
                self.lookPlasm.connect(self.rosSource["output"] >> self.regionFilter["input"], self.regionFilter["output"] >> self.voxelFilter["input"],\
                        self.rosSource["R"] >> self.pclFunsion["R"], self.rosSource["T"] >> self.pclFunsion["T"],\
                        self.voxelFilter["output"] >> self.pclFunsion["input"], self.pclFunsion["output"] >> self.objectFilter["input"], \
                        self.objectFilter["output"] >> self.publisher["input"])


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

                self.cleanPlasm = ecto.Plasm()
                self.cleanPlasm.connect(self.rosSource["output"] >> self.pclFunsion["input"], self.pclFunsion["output"] >> self.objectFilter["input"])

		self.keepRun = True
                #self.handle_clean("init")

	def start(self):
		self.lookService = rospy.Service('look', Look, self.handle_look)
		self.savePclService = rospy.Service('save_pcl', SavePCL, self.handle_save)
		self.loadPclService = rospy.Service('load_pcl', LoadPCL, self.handle_load)
		self.cleanPclService = rospy.Service('clean_pcl', CleanPCL, self.handle_clean)
		self.uploadSceneService = rospy.Service('upload_scene', UploadScene, self.handle_uploadScene)

	def handle_look(self, req):
		print("in look service call back")
		self.sched = ecto.Scheduler(self.lookPlasm)
		lock.acquire()
		self.keepRun = self.sched.execute(niter=1)
		lock.release()
		return LookResponse(0)

	def handle_uploadScene(self, req):
		print("upload the scene file")
		print(os.system(req.fileName))
		return UploadSceneResponse(0)

	def handle_save(self, req):
		print("saving pcl to pcd file")
		print("file name = ", req.fileName)
		self.pclFunsion.params.save = True
		self.sched = ecto.Scheduler(self.savePlasm)
		lock.acquire()
		self.keepRun = self.sched.execute(niter=1)
		lock.release()
		self.pclFunsion.params.save = False
		return SavePCLResponse(0)

	def handle_load(self, req):
		print "file name = ", req.fileName
		self.sched = ecto.Scheduler(self.loadPlasm)
		lock.acquire()
		self.sched.execute(niter=1)
		lock.release()
		return LoadPCLResponse(0)

	def handle_clean(self, req):
                print "cleannig point cloude"
                self.pclFunsion.params.clean = True
                self.sched = ecto.Scheduler(self.cleanPlasm)
		lock.acquire()
                self.keepRun = self.sched.execute(niter=1)
		lock.release()
                self.pclFunsion.params.clean = False
                return CleanPCLResponse(0)

def exit(signum, frame):
	print('Stopping perception bridge')
	exit()

if __name__=="__main__":
	rospy.init_node("perception_bridge")
	ecto_ros.init(sys.argv, "ros_perception")
	p = PerceptionSever()
	p.start()
	signal.signal(signal.SIGINT, exit)
	while not rospy.is_shutdown():
		signal.signal(signal.SIGINT, exit)
		lock.acquire()
		if not p.keepRun:
			print('Stopping perception bridge')
			exit()	
		lock.release()	
		time.sleep(0.5)
