package org.usfirst.frc.team5679.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot 
{
	Talon 		leftMotor0 				= new Talon(2);
	Talon 		leftMotor1 				= new Talon(3);
	Talon 		rightMotor0 			= new Talon(0);
	Talon 		rightMotor1 			= new Talon(1);
	CANTalon 	carriageMotor 			= new CANTalon(3);
	CANTalon	clawMotor				= new CANTalon(2);
	Joystick 	driveJoystick 			= new Joystick(0);
	Joystick 	carriageJoystick 		= new Joystick(1);
	RobotDrive drive = new RobotDrive(leftMotor0, leftMotor1, rightMotor0, rightMotor1);
	DigitalInput limitSwitchTop = new DigitalInput(4);
	DigitalInput limitSwitchBottom = new DigitalInput(5);
	DigitalInput limitSwitchOpen = new DigitalInput(8);
	DigitalInput limitSwitchClosed = new DigitalInput(9);
	AnalogGyro gyro = new AnalogGyro(0);
	BuiltInAccelerometer accel = new BuiltInAccelerometer();
	Encoder rightEncoder = new Encoder(0, 1, false, EncodingType.k4X);
	Encoder leftEncoder = new Encoder(2, 3, false, EncodingType.k4X);
	//encoder is not being used but is wired up
	//	Encoder clawEncoder = new Encoder(6, 7, false, EncodingType.k1X);
	//	int clawEncoderPulses = 124;
	
	static final double startingAngle = 0;
	static final double Kp = .02;
	static final double speedFactor = 1;
	static final double clawSpeedFactor = .5;
	static final double carriageSpeedFactor = .6;
	double speedAdjust = 1;
	boolean runOnce = true;
	boolean reverse = false;
	int stepToPerform = 0;
	static final double driveOffset = .9;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		rightEncoder.setDistancePerPulse(Math.PI * .5 / 250);
		leftEncoder.setDistancePerPulse(Math.PI * .5 / 250);

		SmartDashboard.putString("robot init", "robot init");

		rightEncoder.reset();
		leftEncoder.reset();
//		clawEncoder.reset();

	}

	/**
	 * This function sets up any necessary data before the autonomous control
	 * loop.
	 */
	public void autonomousinit() {

		SmartDashboard.putString("autonomous init", "autonomous init");
		gyro.reset();
		gyro.setSensitivity(.007);
		gyro.setPIDSourceType(PIDSourceType.kRate);
	}

	/**
	 * This function is called periodically during autonomous control
	 */
	public void autonomousPeriodic() {
//		double angle = gyro.getAngle();
		boolean nextStep = false;

		switch (stepToPerform) {
		case 0:
			nextStep = moveBase(10, 0.5, 0);
			break;
//		case 1:
//			nextStep = controlClaw(.6);
//			try {
//				Thread.sleep(2000);
//			} catch (InterruptedException e1) {
//				// TODO Auto-generated catch block
//				e1.printStackTrace();
//			}
//			controlClaw(0);
//			break;
//		case 1:
//			nextStep = moveCarriage(-0.5);
//			try {
//				Thread.sleep(3000);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//			moveCarriage(0);
//		 	break;
//		case 2:
//			nextStep = moveCarriage(0.5);
//			try {
//				Thread.sleep(3000);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//			moveCarriage(0);
//		 	break;
//		case 0:
//			nextStep = turnBase(.6, 90);
//			break;
//		case 1:
//			nextStep = turnBase(.6, 180);
//			break;
//		case 5:
//			nextStep = controlClaw(-.6);
//			try {
//				Thread.sleep(3000);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//			controlClaw(0);
//			break;
		}

		if (nextStep) {
			stepToPerform++;
		}
		
		debug();
	}

	public void debug(){
		SmartDashboard.putNumber("AccelX", accel.getX());
		SmartDashboard.putNumber("AccelY", accel.getY());
		SmartDashboard.putNumber("AccelZ", accel.getZ());
		SmartDashboard.putNumber("Joystick x", driveJoystick.getX());
		SmartDashboard.putNumber("Joystick y", driveJoystick.getY());
		SmartDashboard.putBoolean("Upper Limit", limitSwitchTop.get());
		SmartDashboard.putBoolean("Lower Limit", limitSwitchBottom.get());
		SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", -1 * leftEncoder.getDistance());
//		SmartDashboard.putNumber("clawEncoder", clawEncoder.get());
		SmartDashboard.putNumber("Claw 3 Value", carriageJoystick.getRawAxis(1));
		SmartDashboard.putBoolean("Claw Open Limit", limitSwitchOpen.get());
		SmartDashboard.putBoolean("Claw Close Limit", limitSwitchClosed.get());
	}
	
	
	/**
	 * This function is for moving forward a set number of feet. Returns a
	 * boolean indicating whether the movement is complete.
	 */
	public boolean moveBase(double feet, double speed, double angle) {
		if (rightEncoder.getDistance() >= feet
				|| leftEncoder.getDistance() >= feet) {
			drive.tankDrive(0, 0);
			return true;
		} else {
			drive.tankDrive(speed, speed * driveOffset);
			return false;
		}
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
			drive.tankDrive(0, 0);;
			return true;
		}
	}

	/**
	 * This function is for moving the carriage.
	 */
	public boolean moveCarriage(double speed) {
		boolean moveValid = true;

		if (speed > 0 && limitSwitchTop.get()) {
			moveValid = false;
		} else if (speed < 0 && limitSwitchBottom.get()) {
			moveValid = false;
		}

		if (moveValid) {
			carriageMotor.set(speed);
		} else {
			carriageMotor.set(0);
		}

		return !moveValid;
	}

	/**
	 * This function is for opening and closing the claw.
	 */
	//TODO check that it's moving the correct direction and that the pulses returned are correct
	public boolean controlClaw(double speed) {
		boolean moveValid = true;
		
		if (speed > 0 && limitSwitchOpen.get()) {
			moveValid = false;
		} else if (speed < 0 && limitSwitchClosed.get()) {
			moveValid = false;
		}

		if (moveValid) {
			clawMotor.set(speed);
			SmartDashboard.putNumber("Claw Speed", speed);
		} else {
			clawMotor.set(0);
		}

		return !moveValid;
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		double LP = -driveJoystick.getRawAxis(1);
		double RP = -driveJoystick.getRawAxis(5);
		double UD = carriageJoystick.getRawAxis(1);
		double clawControl = carriageJoystick.getRawAxis(3);
		boolean moveValidCarriage = true;
		boolean moveValidClaw = true;
		double speedAdj = driveJoystick.getThrottle();

		if(driveJoystick.getRawAxis(3) > 0.2)
			speedAdjust = 1;
		else if(driveJoystick.getRawAxis(2) > 0.2)
			speedAdjust = .5;
		else
			speedAdjust = .7;
		
		if (Math.abs(LP) < 0.1) {
			LP = 0;

			if (Math.abs(RP) < 0.1) {
				RP = 0;
			}
		}

		setRobotDriveSpeed(drive, LP * speedAdjust, RP * speedAdjust);

		//Carriage Control
		if (Math.abs(UD) < 0.3) {
			UD = 0;
		}

		UD = UD * carriageSpeedFactor;
		
		if (UD > 0 && limitSwitchTop.get()) {
			moveValidCarriage = false;
		} else if (UD < 0 && limitSwitchBottom.get()) {
			moveValidCarriage = false;
		}

		if (moveValidCarriage) {
			setCANTalonSpeed(carriageMotor, UD);
		} else {
			setCANTalonSpeed(carriageMotor, 0);
		}
		
		//Claw Control
		if (Math.abs(clawControl) < .3) {
			clawControl = 0;
		}

	clawControl =	clawControl * clawSpeedFactor;
		
		if (clawControl > 0 && limitSwitchOpen.get()) {
			moveValidClaw = false;
		} else if (clawControl < 0 && limitSwitchClosed.get()) {
			moveValidClaw = false;
		}

		if (moveValidClaw) {
			setCANTalonSpeed(clawMotor, clawControl);
		} else {
			setCANTalonSpeed(clawMotor, 0);
		}
		
		debug();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
	}

	/**
	 * This method sets the speed and applies the limiting speed factor for
	 * CANTalons
	 * 
	 * @param motorkkk  x lo
	 * @param speed
	 */
	// TODO: ADD ACCELERATION CODE
	public void setCANTalonSpeed(CANTalon motor, double speed) {
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
	// TODO: ADD ACCELERATION CODE
	public void setRobotDriveSpeed(RobotDrive driveTrain, double leftSpeed,
			double rightSpeed) {
		driveTrain.tankDrive(leftSpeed * speedFactor, rightSpeed * speedFactor);

	}
}
