RobotName: # Robot Name
Johnny5

Type: # Robot type
Johnny5

DriveHandler: # Robot default drive handler with default argument values
share.Drive.DifferentialDriveHandler()

InitHandler: # Robot default init handler with default argument values
Johnny5.Johnny5InitHandler(comPort='/dev/tty.usbserial-A600eIiI')

LocomotionCommandHandler: # Robot default locomotion command handler with default argument values
Johnny5.Johnny5LocomotionCommandHandler()

MotionControlHandler: # Robot default motion control handler with default argument values
share.MotionControl.VectorControllerHandler()

PoseHandler: # Robot default pose handler with default argument values
share.Pose.ViconPoseHandler(host="10.0.0.102",port=800,x_VICON_name="Johnny5:Johnny5 <t-X>",y_VICON_name="Johnny5:Johnny5 <t-Y>",theta_VICON_name="Johnny5:Johnny5 <a-Z>")

SensorHandler: # Robot default sensor handler with default argument values
Johnny5.Johnny5SensorHandler()

ActuatorHandler: # Robot default actuator handler with default argument values
Johnny5.Johnny5ActuatorHandler()
