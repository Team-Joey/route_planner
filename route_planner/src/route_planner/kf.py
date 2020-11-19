from 	geometry_msgs.msg 	import Pose, PoseArray, Quaternion, PoseWithCovariance
from 	kf_base 		import KFLocaliserBase
from 	util 			import rotateQuaternion, getHeading
from 	time 			import time
import 	numpy 			as np
import 	random
import 	math
import 	rospy

class KFLocaliser(KFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(KFLocaliser, self).__init__()

        # ----- Set motion model parameters
	self.ODOM_ROTATION_NOISE 	= 0.05
	self.ODOM_TRANSLATION_NOISE 	= 0.05
	self.ODOM_DRIFT_NOISE 		= 0.05

	# ----- Set Variance parameters initial values
	self.VAR_SENSOR 	= [0.25, 0.75, 0.25]		# Q - measurement noise
	self.VAR_MOTION 	= [0.25, 0.25, 0.50]		# R - motion noise 
	self.variance 		= [0.1, 0.1, 0.1]
       
#--------------------------------------------------------------------------------------------------------------------------------------------------------

    def initialise_kalman_filter(self, initialpose):


        """
	The method sets the initial pose of the robot as the mean
	of the Gaussian distribution and gets the initial variance
	of each parameter as 1. 
        """ 

	self.mean[0] 		= initialpose.pose.pose.position.x
	self.mean[1] 		= initialpose.pose.pose.position.y
	self.mean[2] 		= initialpose.pose.pose.orientation
	
	self.variance[0] 	= self.estimatedpose.pose.covariance[0]
	self.variance[1] 	= self.estimatedpose.pose.covariance[7]
	self.variance[2] 	= self.estimatedpose.pose.covariance[35]

#--------------------------------------------------------------------------------------------------------------------------------------------------------
    def catch_nan(self, scan):

        """ 
	Remove NaN valued scan ranges- if NaN value is found, set range to 
	max scan range
        """ 
        noisefilteredranges = []
        for i in range(0, len(scan.ranges)):
            if math.isnan(scan.ranges[i]):
                    noisefilteredranges.append(scan.range_max)
            else:
                noisefilteredranges.append(scan.ranges[i])

        scan.ranges = noisefilteredranges

        return scan
#--------------------------------------------------------------------------------------------------------------------------------------------------------
    def kalman_update(self, scan):
	
	
        """
        This method performs the Correction step of the Kalman
	filter algorithm. It updates the mean and variance based
	on 

	The Prediction step is done in the predict_from_odometry 
	method in the base_kf class. In this method we only 
	update the variance by adding motion noise and re-
	calculate the sensor noise based on our confidence
	in our position.
        """
	# ----- Remove nan values from the scan reading we get
	scan = self.catch_nan(scan)

	# ----- Calculate the predicted variance by adding motion noise (R)
	self.variance[0] = self.variance[0] + self.VAR_MOTION[0]
	self.variance[1] = self.variance[1] + self.VAR_MOTION[1]
	self.variance[2] = self.variance[2] + self.VAR_MOTION[2]
	
	# ----- Re-calculate the sensor noise based on our confidence in our position
	p = Pose()
	p.position.x = self.mean[0]
	p.position.y = self.mean[1]
	p.orientation = self.mean[2]

	scan_weight = self.sensor_model.get_weight(scan, p)

	self.VAR_SENSOR[0] = 1/scan_weight
	self.VAR_SENSOR[1] = 1/scan_weight
	self.VAR_SENSOR[2] = 1/scan_weight

	#  ----- Correction step 
	observation = self.sensor_model.get_observation_mean(scan, p)

	K_gain = [1, 1, 1]

	# ----- Calculate the K_gain: K_gain = self.variance*(inv(self.variance + VAR_SENSOR))
	K_gain[0] = self.variance[0]/(self.variance[0] + self.VAR_SENSOR[0])
	K_gain[1] = self.variance[1]/(self.variance[1] + self.VAR_SENSOR[1])
	K_gain[2] = self.variance[2]/(self.variance[2] + self.VAR_SENSOR[2])

	# ----- Calculate self.mean = new_mean + K_gain*(observation - new_mean) 

	self.mean[0] = self.mean[0] + K_gain[0]*(observation[0] - self.mean[0])
	self.mean[1] = self.mean[1] + K_gain[1]*(observation[1] - self.mean[1])

	rotateby = getHeading(observation[2]) - getHeading(self.mean[2]) 
	rotateby = rotateby * K_gain[2]
	self.mean[2] = rotateQuaternion(self.mean[2], rotateby)


	# ----- Calculate the new variance: self.variance = self.variance*(np.indentity - K_gain)

	self.variance[0] = self.variance[0]*(1 - K_gain[0])
	self.variance[1] = self.variance[1]*(1 - K_gain[1])
	self.variance[2] = self.variance[2]*(1 - K_gain[2])

#--------------------------------------------------------------------------------------------------------------------------------------------------------
    def sample_distribution(self):
	"""
	This method samples x, y and orientation from a normal distribution,
	using the current mean and variance.
	"""
        x = random.normalvariate(self.mean[0], self.variance[0])
        y = random.normalvariate(self.mean[1], self.variance[1])

        rotateby = random.normalvariate(getHeading(self.mean[2]), self.variance[2]) - getHeading(self.mean[2])
        quat = rotateQuaternion(self.mean[2], rotateby)

        return [x,y,quat]

    def estimate_pose(self):	
        """
	This method creates a new PoseWithCovariance object, which uses 
	the current mean values as the Pose (the position we have the 
	highest confidence in) and the current Variance values to calculate
	the variance of x, y and the orientation, as well as the covariance 
	for x,y.
        This should calculate and return an updated robot pose estimate based
        on the current Gaussian distribution.
        
        """

	ePose = PoseWithCovariance()
    	ePose.pose.position.x  = self.mean[0]
    	ePose.pose.position.y  = self.mean[1]
	ePose.pose.orientation = self.mean[2]
	covariance = 	[ 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0]
	
	array = self.sample_distribution()

	covariance[0]  = self.variance[0]
	covariance[1]  = (array[0] - self.mean[0])*(array[1] - self.mean[1])
	covariance[6]  = covariance[1]
	covariance[7]  = self.variance[1]
	covariance[35] = self.variance[2]

	ePose.covariance = covariance
	
	return ePose	


