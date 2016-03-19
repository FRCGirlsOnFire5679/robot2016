package org.usfirst.frc.team5679.robot;

import java.util.Date;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.RawData;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {
	Victor leftMotor0 = new Victor(0);
	Victor leftMotor1 = new Victor(1);
	Victor rightMotor0 = new Victor(2);
	Victor rightMotor1 = new Victor(3);
	Victor victorsBeltLeft = new Victor(4);
	Victor victorsBeltRight = new Victor(5);
	Joystick driveJoystick = new Joystick(0);
	Joystick firingJoystick = new Joystick(1);
	RobotDrive drive = new RobotDrive(leftMotor0, leftMotor1, rightMotor0,
			rightMotor1);
	DigitalInput firingLimitSwitch = new DigitalInput(0);
	AnalogGyro gyro = new AnalogGyro(0);
	BuiltInAccelerometer accel = new BuiltInAccelerometer();
	Encoder rightEncoder = new Encoder(3, 4, false, EncodingType.k4X);
	Encoder leftEncoder = new Encoder(1, 2, false, EncodingType.k4X);
	NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);
	CameraServer camera;
	//Image frame;
	int session;
	RawData colorTable;
	private int autonomousMode = 0; // initialize default mode
	SendableChooser autoChooser;
	String cameraDesc = "Front";
	
	private String cameraName = "cam0";
	USBCamera targetCam = new USBCamera(cameraName);
	NIVision.Image frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
	
	static final double startingAngle = 0;
	static final double Kp = .02;
	static final double speedFactor = 1;
	static final double firingSpeedFactor = 1;
	static final double driveOffset = .98;
	// Adjust this value down for more distance in autonomous, up for less distance
	static final double wheelCircumference = 1.43;
	static final double encoderPulses = 250;
	static final double distancePerPulse = wheelCircumference / encoderPulses;
	static final double halfSpeed = .5;
	static final double minJoystickValue = 0.2;
	static final double minimumSpeed = 0.1;
	static final int imageQuality = 50;
	static final int fullSpeed = 1;
	static final double firingMaxDistance = 1;
	static final String imageFileName = "/camera/image.jpg";

	double speedAdjust = 1;
	double previousFireSpeed = 0;
	boolean runOnce = true;
	boolean reverse = false;
	int stepToPerform = 0;
	long startTime;
	long fireTime = 5000;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		rightEncoder.setDistancePerPulse(distancePerPulse);
		leftEncoder.setDistancePerPulse(distancePerPulse);

		SmartDashboard.putString("robot init", "robot init");

		rightEncoder.reset();
		leftEncoder.reset();
		
		autoChooser = new SendableChooser();
		autoChooser.addDefault("Drive", 0);
		autoChooser.addObject("Drive and Fire", 1);
		SmartDashboard.putData("Autonomous mode chooser", autoChooser);
		SmartDashboard.putString("Camera", cameraDesc);
		//SmartDashboard.putBoolean("Captured", !firingLimitSwitch.get());

//		 camera = CameraServer.getInstance();
//		 camera.setQuality(30);
////		 the camera name (ex "cam0") can be found through the roborio web
////		 interface
//		 camera.startAutomaticCapture("cam0");
		 try {
			targetCam.openCamera();	
			targetCam.startCapture();		 
		 } catch (Exception e) {}
	}

	/**
	 * This function sets up any necessary data before the autonomous control
	 * loop.
	 */
	public void autonomousinit() {
		rightEncoder.reset();
		leftEncoder.reset();
		SmartDashboard.putString("autonomous init", "autonomous init");
//		gyro.reset();
//		gyro.setSensitivity(.007);
//		gyro.setPIDSourceType(PIDSourceType.kRate);
		stepToPerform = 0;
//		autonomousCommand = (Command) autoChooser.getSelected();
//		autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous control
	 */
	@Override
	public void autonomousPeriodic() {
		autonomousMode = (int) autoChooser.getSelected();
		
		boolean nextStep = false;
		int distance = 16;
		double speed = .7;
		switch (autonomousMode) {
			// Mode 0, drive
			case 0:
				moveBase(distance, speed, 0);
				break;
			// Mode 1, Drive and fire
			case 1: 
				switch (stepToPerform) {
					case 0:
						// adjust the first number in the movebase call for number of feet to move in autonomous
						nextStep = moveBase(distance, speed, 0);
						startTime = System.currentTimeMillis();
						break;
					case 1:
						nextStep = fire();
						break;
				}
	
				if (nextStep) {
					stepToPerform++;
				}
				
				break;
		}

		debug();
	}

	public void debug() {
//		SmartDashboard.putNumber("AccelX", accel.getX());
//		SmartDashboard.putNumber("AccelY", accel.getY());
//		SmartDashboard.putNumber("AccelZ", accel.getZ());
//		SmartDashboard.putNumber("Joystick x", driveJoystick.getX());
//		SmartDashboard.putNumber("Joystick y", driveJoystick.getY());
//		SmartDashboard.putBoolean("Limit", firingLimitSwitch.get());
//		SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
//		SmartDashboard.putNumber("Left Encoder", -1 * leftEncoder.getDistance());
	}

	/**
	 * This function is for moving forward a set number of feet. Returns a
	 * boolean indicating whether the movement is complete.
	 */
	public boolean moveBase(double feet, double speed, double angle) {
		if (rightEncoder.getDistance() >= feet
				|| leftEncoder.getDistance() >= feet) {
			drive.tankDrive(-.2, -.2);
			drive.tankDrive(0, 0);
			return true;
		} else {
			drive.tankDrive(speed, speed * driveOffset);
			return false;
		}
	}
	
	/**
	 * This function is for firing the boulder for a given amount of time. Returns
	 * a boolean indicating whether the movement is complete.
	 */
	public boolean fire() {
		setVictorSpeed(victorsBeltLeft, -fullSpeed);
		setVictorSpeed(victorsBeltRight, fullSpeed);
		return true;
	}
	
	public void changeCamera() {
		if (cameraName == "cam0") {
			cameraName = "cam4";
			cameraDesc = "Back";
		}
		else {
			cameraName = "cam0";
			cameraDesc = "Front";
		}

		SmartDashboard.putString("Camera", cameraDesc);
				
		try {
			targetCam.stopCapture();
			targetCam.closeCamera();
			
			targetCam = new USBCamera(cameraName);
			
			targetCam.openCamera();
			targetCam.startCapture();			
		} catch (Exception e) {}
	}

	/**
	 * This function is for turning the base at a given speed and angle. Returns
	 * a boolean indicating whether the movement is complete.
	 */
	public boolean turnBase(double speed, double desiredAngle) {
		double currentAngle = gyro.getAngle();
		double angleDifference = currentAngle - startingAngle;
		if ((angleDifference > 0 && angleDifference <= desiredAngle)
				|| (angleDifference < 0 && angleDifference >= desiredAngle)) {
			drive.tankDrive(speed, -speed * driveOffset);
			return false;
		} else {
			drive.tankDrive(0, 0);
			;
			return true;
		}
	}

	/**
	 * grab an image, draw the circle, and provide it for the camera server
	 * which will in turn send it to the dashboard.
	 */
	public void operatorControl() {
//		NIVision.IMAQdxStartAcquisition(session);
//
//		while (isOperatorControl() && isEnabled()) {
//
//			NIVision.IMAQdxGrab(session, frame, 1);
//			NIVision.imaqDrawShapeOnImage(frame, frame, rect,
//					DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
//
//			CameraServer.getInstance().setImage(frame);
//
////			if (snapshotButton) {
////				NIVision.imaqWriteJPEGFile(frame, imageFileName, imageQuality,
////						colorTable);
////			}
//
//			/** robot code here! **/
//			Timer.delay(0.005); // wait for a motor update time
//		}
//		NIVision.IMAQdxStopAcquisition(session);
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		rightEncoder.reset();
		leftEncoder.reset();
		double LP = -driveJoystick.getRawAxis(1);
		double RP = -driveJoystick.getRawAxis(5);
		boolean fireButton = firingJoystick.getRawButton(1);
		boolean intakeButton = firingJoystick.getRawButton(2);
		boolean cameraButton = driveJoystick.getRawButton(5);
		
		if (cameraButton) {
			changeCamera();
		}
		
		if (driveJoystick.getRawAxis(3) > minJoystickValue){
			speedAdjust = fullSpeed;
		}
		else if (driveJoystick.getRawAxis(2) > minJoystickValue) {
			speedAdjust = halfSpeed;
		}
		else {
			speedAdjust = .7;
		}
		
		if (Math.abs(LP) < minimumSpeed) {
			LP = 0;

			if (Math.abs(RP) < minimumSpeed) {
				RP = 0;
			}
		}
		
		setRobotDriveSpeed(drive, LP * speedAdjust, RP * speedAdjust);
				
		if (fireButton && !intakeButton) {
			setVictorSpeed(victorsBeltLeft, -fullSpeed);
			setVictorSpeed(victorsBeltRight, fullSpeed);
			//SmartDashboard.putBoolean("Captured", false);
		} else if (intakeButton && !fireButton) { 
			if (firingLimitSwitch.get()){
				setVictorSpeed(victorsBeltLeft, fullSpeed);
				setVictorSpeed(victorsBeltRight, -fullSpeed);	
			} else {
				setVictorSpeed(victorsBeltLeft, 0);
				setVictorSpeed(victorsBeltRight, 0);
				//SmartDashboard.putBoolean("Captured", true);	
			}
		} else {
			setVictorSpeed(victorsBeltLeft, 0);
			setVictorSpeed(victorsBeltRight, 0);
			//SmartDashboard.putBoolean("Captured", true);	
		}
		
		targetCam.getImage(frame);
		//targetCam.setFPS(fps);
		//targetCam.setBrightness(brightness);
		
		try {
			CameraServer.getInstance().setImage(frame);			
		} catch (Exception e) {}
	}

	/**
	 * This method sets the speed and applies the limiting speed factor for
	 * Victors
	 * 
	 * @param motor
	 * @param speed
	 */
	public void setVictorSpeed(Victor motor, double speed) {
		motor.set(speed * speedFactor);
	}

	/**
	 * This method sets the speed and applies the limiting speed factor for
	 * robot Tank Drive
	 * 
	 * @param driveTrain
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void setRobotDriveSpeed(RobotDrive driveTrain, double leftSpeed,
			double rightSpeed) {
		driveTrain.tankDrive(leftSpeed * speedFactor, rightSpeed * speedFactor);
	}
}
