#!/usr/bin/env python

import rospy
import rospkg
from openravepy import *
from sympy import *
from sympy.physics.units import *


from numpy import *
import time
from openravepy.misc import InitOpenRAVELogging 
InitOpenRAVELogging() 


class Rave():
	def __init__(self):

		self.path = rospkg.RosPack().get_path('OpenRaveTutorials')
		#print path
		self.env = Environment()
		self.env.SetViewer('qtcoin')
		
		



	def Transform(self):


		#Transforming the body
		Tz = matrixFromAxisAngle([0,0,np.pi/4])
		#print Tz
		self.env.Load(self.path + '/models/robots/barrett_wam.dae')
		robot = self.env.GetRobots()[0]

		with self.env: # lock the environment since robot will be used
			for body in self.env.GetBodies():
				body.SetTransform(np.dot(Tz,body.GetTransform()))



		try:
			while True:
				n=1
		except KeyboardInterrupt:
			print "exiting"


	def CreatAndUpdate(self):


		#creating and updating a body

		with self.env:
			body = RaveCreateKinBody(self.env,'')
			body.SetName('testbody')
			body.InitFromBoxes(numpy.array([[1,1,1,0.1,0.2,0.3]]),True)
			self.env.AddKinBody(body)


		time.sleep(5)


		with self.env:
			self.env.Remove(body)
			body.InitFromBoxes(numpy.array([[-0.4,0,0,0.1,0.2,0.3],[0.4,0,0,0.1,0.2,0.9]]),True) # set geometry as two boxes
			self.env.AddKinBody(body)


		time.sleep(5)


	def Planner(self):

		#Using a BiRRT Planner
		self.env.Load('data/lab1.env.xml')
		robot = self.env.GetRobots()[0]
		RaveSetDebugLevel(DebugLevel.Debug)
		manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
		# call motion planner with goal joint angles
		manipprob.MoveManipulator(goal=[-0.75,1.24,-0.064,2.33,-1.16,-1.548,1.19]) 
		robot.WaitForController(0) # wait

	def InverKin(self):
		self.env.Load('data/pr2test1.env.xml')
		robot = self.env.GetRobots()[0]

		manip = robot.SetActiveManipulator('rightarm_torso')
		ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,
			iktype=IkParameterization.Type.Transform6D)

		if not ikmodel.load():
			ikmodel.autogenerate()

		with self.env:
			Tgoal = numpy.array([[0,-1,0,-0.21],[-1,0,0,0.04],[0,0,-1,0.92],[0,0,0,1]])
			sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions)
			with robot:
				robot.SetDOFValues(sol,manip.GetArmIndices())
				Tee = manip.GetEndEffectorTransform()
				self.env.UpdatePublishedBodies()
				raw_input('press any key')

			raveLogInfo('Tee is: '+ str(Tee))


	def InverKin3D(self):
		self.env.Load('data/katanatable.env.xml')
		robot = self.env.GetRobots()[0]


		manip = robot.GetActiveManipulator()
		ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,
			iktype=IkParameterization.Type.Translation3D)

		if not ikmodel.load():
			ikmodel.autogenerate()

		with robot:
			robot.SetDOFValues([2.58, 0.547, 1.5, -0.7],[0,1,2,3]) # set the first 4 dof values
    		Tee = manip.GetEndEffectorTransform() # get end effector
    		ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
    		sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions

		h = self.env.plot3(Tee[0:3,3],10) # plot one point
		with robot: # save robot state
			for sol in sols[::10]: # go through every 10th solution
				robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
				self.env.UpdatePublishedBodies() # allow viewer to update new robot
        		raw_input('press any key')

		raveLogInfo('restored dof values: '+repr(robot.GetDOFValues())) # robot state is restored to original


	def PlanEE(self):
		self.env.Load('data/pr2test1.env.xml')
		robot = self.env.GetRobots()[0]
		robot.SetActiveManipulator('leftarm_torso')
		ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,
			iktype=IkParameterization.Type.Transform6D)
		if not ikmodel.load():
			ikmodel.autogenerate()

		manipprob = interfaces.BaseManipulation(robot)
		Tgoal = numpy.array([[0,-1,0,-0.21],[-1,0,0,0.04],[0,0,-1,0.92],[0,0,0,1]])
		res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10)
		robot.WaitForController(0)


	def mainn(self):
		self.env.Load('robots/barrettwam.robot.xml')
		robot1 = self.env.GetRobots()[0]

		with self.env: # lock the environment since robot will be used
			ray1 = array((0,0,-10,0,0,100)) # specify dir*range, pos
        	ray2 = array((0,0,10,0,0,-100)) # specify dir*range, pos
        	inliers,hitpoints = self.env.CheckCollisionRays(r_[[ray1],[ray2]],robot1)
        	print 'rays hit:',inliers,'hit points:',hitpoints

        	report1 = CollisionReport()
        	inlier1 = self.env.CheckCollision(Ray(ray1[0:3],ray1[3:6]))
        	#inlier1 = self.env.CheckCollision(Ray(ray1[0:3],ray1[3:6]),report1)
        	print 'numcontacts: ',len(report1.contacts)
        	print ' pos: ', report1.contacts[0].pos
        	print ' norm: ',report1.contacts[0].norm
		



		try:
			while True:
				n=1
		except KeyboardInterrupt:
			print "exiting"


		

		


	








if __name__ == '__main__':
	Or= Rave()
	#Or.Transform()
	#Or.CreatAndUpdate()
	#Or.Planner()
	#Or.InverKin()
	#Or.InverKin3D()
	#Or.PlanEE()
	Or.mainn()
