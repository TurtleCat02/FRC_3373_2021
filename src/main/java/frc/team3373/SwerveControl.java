package frc.team3373;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.util.MathUtil;

public class SwerveControl {

	public static enum Side {
		NORTH, SOUTH, EAST, WEST, UNKNOWN;
	}

	public static enum DriveMode {
		ROBOTCENTRIC, FIELDCENTRIC, OBJECTCENTRIC;
	}

	private SwerveWheel FLWheel;
	private SwerveWheel BLWheel;
	private SwerveWheel FRWheel;
	private SwerveWheel BRWheel;

	private SwerveWheel[] wheelArray;

	// private double distanceToCenter;
	private double distanceToFront;

	private double robotLength;
	private double robotWidth;

	private double orientationOffset = 0;
	private double frontDirectionOffset = 0;

	private boolean isFieldCentric = false;
	private boolean isObjectCentric = false;

	private long getTargetAngleDelay = System.currentTimeMillis();

	private double maxTargetSpeed = 0.45;
	private double targetRobotAngle = 0;

	private SuperAHRS ahrs;

	private byte controllerMode = 1;

	private PIDController rotationPID;

	// Values for rotating around a point
	private double[] rotateAngles;
	private double[] rotateRatios;
	private boolean rotateMode;
	private double rotatePointX;
	private double rotatePointY;

	private static SwerveControl instance;

	private final double hPI = Math.PI / 2.0; // Pi / 2
	private final double hWidth = Constants.robotWidth / 2.0;
	private final double hLength = Constants.robotLength / 2.0;

	public static SwerveControl getInstance() {
		if (instance == null) {
			instance = new SwerveControl();
		}
		return instance;
	}

	// public SwerveControl(int LFrotateMotorID, int LFdriveMotorID, int LFEncMin,
	// int LFEncMax, int LFEncHome,
	// int LBrotateMotorID, int LBdriveMotorID, int LBEncMin, int LBEncMax, int
	// LBEncHome, int RFrotateMotorID,
	// int RFdriveMotorID, int RFEncMin, int RFEncMax, int RFEncHome, int
	// RBrotateMotorID, int RBdriveMotorID,
	// int RBEncMin, int RBEncMax, int RBEncHome, SuperAHRS AHRS, double width,
	// double length) {

	public SwerveControl() {
		robotWidth = Constants.robotWidth;
		robotLength = Constants.robotLength;
		double robotX = robotWidth / 2;
		double robotY = robotLength / 2;

		double rotAngle = Math.atan(hWidth / hLength);
		System.out.println("Rotational Offset: " + Math.toDegrees(rotAngle));

		rotationPID = new PIDController(0.005, 0.005, 0);
		initPIDController();

		ahrs = SuperAHRS.getInstance();

		FLWheel = new SwerveWheel("FrontLeft", Constants.FLRotateMotorID, Constants.FLDriveMotorID, Constants.FLEncMin,
				Constants.FLEncMax, Constants.FLEncHome, Constants.relativeEncoderRatio,
				Math.atan(robotX / robotY) + Math.PI);
		FRWheel = new SwerveWheel("FrontRight", Constants.FRRotateMotorID, Constants.FRDriveMotorID, Constants.FREncMin,
				Constants.FREncMax, Constants.FREncHome, Constants.relativeEncoderRatio,
				Math.atan(-robotX / robotY) + Math.PI);
		BLWheel = new SwerveWheel("BackLeft", Constants.BLRotateMotorID, Constants.BLDriveMotorID, Constants.BLEncMin,
				Constants.BLEncMax, Constants.BLEncHome, Constants.relativeEncoderRatio, Math.atan(-robotX / robotY));
		BRWheel = new SwerveWheel("BackRight", Constants.BRRotateMotorID, Constants.BRDriveMotorID, Constants.BREncMin,
				Constants.BREncMax, Constants.BREncHome, Constants.relativeEncoderRatio, Math.atan(robotX / robotY));

		setRotatePoint(0, 0); // Set rotate point to center of robot

		FLWheel.setPIDController(Constants.ROTATE_FL_PID);
		FRWheel.setPIDController(Constants.ROTATE_FR_PID);
		BLWheel.setPIDController(Constants.ROTATE_BL_PID);
		BRWheel.setPIDController(Constants.ROTATE_BR_PID);

		wheelArray = new SwerveWheel[] { FLWheel, BLWheel, BRWheel, FRWheel };

		resetOrentation();
		recalculateWheelPosition();
	}

	// ###################################
	// ########### Autonomous ############
	// ###################################

	/**
	 * Resets/initializes the PID controller
	 */
	private void initPIDController() {
		rotationPID.reset();
		rotationPID.enableContinuousInput(-180, 180);
		rotationPID.setTolerance(0.5);
	}

	/**
	 * moves the robot in a direction for a certain amount of time
	 * 
	 * @param angle the angle in degrees to drive at (0,360)
	 * @param speed the speed the drive at (0,1)
	 * @param time  the time in seconds to drive for
	 */
	public void relativeMoveRobot(double angle, double speed, double time) {
		double oldSpeed = maxTargetSpeed;
		setDriveSpeed(speed);
		calculateSwerveControl(Math.sin(Math.toRadians(angle)), -Math.cos(Math.toRadians(angle)), 0);
		try {
			Thread.sleep((long) (time * 1000));
		} catch (Exception e) {
			// Do nothing
		}
		calculateSwerveControl(0, 0, 0);
		setDriveSpeed(oldSpeed);
	}

	public void stop() {
		calculateSwerveControl(0, 0, 0);
		setDriveSpeed(maxTargetSpeed);
	}

	/**
	 * Rotates the robot a relative amount
	 * 
	 * @param angle The amount to rotate by
	 */
	public void relativeRotateRobot(double angle) {
		// SmartDashboard.putNumber("Delta Angle", angle);
		double currentAngle = ahrs.getYaw();
		SmartDashboard.putNumber("Current Angle:", currentAngle);
		double targetAngle = currentAngle + angle;

		if (targetAngle >= 180) {
			targetAngle -= 360;
		} else if (targetAngle < -180) {
			targetAngle += 360;
		}
		SmartDashboard.putNumber("Target Angle: ", targetAngle);

		initPIDController();
		// rotationPID.setSetpoint(targetAngle);
		double power = rotationPID.calculate(currentAngle, targetAngle);
		// SmartDashboard.putBoolean("atSetpoint", rotationPID.atSetpoint());
		while (!rotationPID.atSetpoint() && RobotState.isEnabled()) { // waits until we are within range of the angle
			// SmartDashboard.putBoolean("atSetpoint", rotationPID.atSetpoint());
			// rotationPID.setSetpoint(targetAngle); //tells PID loop to go to the
			// targetAngle
			currentAngle = ahrs.getYaw();
			SmartDashboard.putNumber("Current Angle:", currentAngle);
			SmartDashboard.putNumber("Position Error", rotationPID.getPositionError());
			// SmartDashboard.putNumber("Velocity Error", rotationPID.getVelocityError());
			power = rotationPID.calculate(currentAngle);
			// SmartDashboard.putNumber("PID Power: ", power);
			// calculateSwerveControl(0,0,0.2);
			calculateSwerveControl(0, 0, MathUtil.clamp(power, -.75, .75)); // sets the wheels to rotate based off PID
																			// loop
			try {
				Thread.sleep(1);
			} catch (Exception e) {
				// Do nothing
			}
		}
		// SmartDashboard.putBoolean("atSetpoint", rotationPID.atSetpoint());
		calculateSwerveControl(0, 0, 0); // stops robot spinning
		SmartDashboard.putNumber("Current Angle:", currentAngle);
	}

	/**
	 * Used in Autonomous Mode Only, Rotates robot to a certain angle regardless of
	 * robots current position
	 * 
	 * @param targetAngle Angle(degrees) to which the robot will rotate
	 */

	public void absoluteRotateRobot(double targetAngle) {
		double currentAngle = ahrs.getYaw();
		if (targetAngle >= 180) {
			targetAngle -= 360;
		} else if (targetAngle < -180) {
			targetAngle += 360;
		}

		SmartDashboard.putNumber("Target Angle: ", targetAngle);
		initPIDController();
		// rotationPID.setSetpoint(targetAngle);
		double power = rotationPID.calculate(currentAngle, targetAngle);
		// SmartDashboard.putBoolean("atSetpoint", rotationPID.atSetpoint());
		while (!rotationPID.atSetpoint() && RobotState.isEnabled()) { // waits until we are within range of the angle
			// SmartDashboard.putBoolean("atSetpoint", rotationPID.atSetpoint());
			// rotationPID.setSetpoint(targetAngle); //tells PID loop to go to the
			// targetAngle
			currentAngle = ahrs.getYaw();
			SmartDashboard.putNumber("Current Angle:", currentAngle);
			SmartDashboard.putNumber("Position Error", rotationPID.getPositionError());
			// SmartDashboard.putNumber("Velocity Error", rotationPID.getVelocityError());
			power = rotationPID.calculate(currentAngle);
			// SmartDashboard.putNumber("PID Power: ", power);
			// calculateSwerveControl(0,0,0.2);
			calculateSwerveControl(0, 0, MathUtil.clamp(power, -.75, .75)); // sets the wheels to rotate based off PID
																			// loop
			try {
				Thread.sleep(1);
			} catch (Exception e) {
				// Do nothing
			}
		}

		calculateSwerveControl(0, 0, 0); // stops robot spinning
		SmartDashboard.putNumber("Current Angle:", currentAngle);
	}

	// ###############################
	// ########### Teleop ############
	// ###############################

	/**
	 * Controls the swerve drive based on joystick input, called every teleop loop
	 * 
	 * @param LX Left Joystick X axis (-1, 1)
	 * @param LY Left Joystick Y axis (-1, 1)
	 * @param RX Right Joystick X axis (-1, 1)
	 */
	public void calculateSwerveControl(double LX, double LY, double RX) {
		// If the robot is in rotate mode, bypass swerve calculations
		if (rotateMode) {
			pointRotate(RX);
			return;
		}
		LY = -LY; // Inverts joystick LY to match the Cartesian plane
		double magnitude;
		double angle;

		// Deadband
		if (Math.abs(LX) < 0.02) {
			LX = 0;
		}

		if (Math.abs(LY) < 0.02) {
			LY = 0;
		}

		if (Math.abs(RX) < 0.02) {
			// rAxis = 0;
			RX = 0;
		}

		if (LY == 0 && LX == 0 && RX == 0) { // If nothing is pressed, don't move
			stopMoving();
		} else if (RX == 0) { // If only translating, set the angle of the wheels to the angle of the control stick, and the speed to the magnitude of the control stick
			magnitude = Math.sqrt((LY * LY) + (LX * LX));
			angle = Math.atan2(LY, LX);

			// angle = applyControllerLimiter(angle, magnitude);
			SmartDashboard.putNumber("direction", Math.toDegrees(angle));
			for (SwerveWheel wheel : wheelArray) { // Set all wheels to angle and speed
				wheel.setTargetAngle(angle);
				wheel.setSpeed(maxTargetSpeed * magnitude);
			}
		} else if (LY == 0 && LX == 0) { // If only rotating, calculate rotating about the center of the robot
			// System.out.println("RX: " + RX);
			calculateRotateAngles(0, 0);
			pointRotate(RX);
		} else { // If translating and rotating at the same time, move the robot as if it is rotating about a point pi/2 radians to the right of the angle that the left stick is pressed, inversely proportional to the magnitude of the right (rotation) stick
			// System.out.println("1LX: " + LX + " LY: " + LY);
			magnitude = Math.sqrt((LY * LY) + (LX * LX)); // How much the left stick is pressed
			angle = Math.atan2(LY, LX); // The angle that the left stick is pressed
			double minRadius = Math.sqrt((hLength * hLength) + (hWidth * hWidth)) + Config.getNumber("minRadiusOffset", 0); // The minimum distance of the point that it will rotate around from the center of the robot, default is the distance of the wheels
			double pointMagnitude = (minRadius / RX) * Config.getNumber("pointMagnitudeScalar", 1); // Calculated distance of the point from the center of the robot, inversely proportional to the magnitude of the rotation

			calculateRotateAngles(pointMagnitude * Math.cos(angle - hPI), pointMagnitude * Math.sin(angle - hPI)); // Convert (magnitude, angle - pi/2) of the point to (x, y)
			System.out.println("Rotate Point: (" + (pointMagnitude * Math.cos(angle + hPI)) + "," + pointMagnitude * Math.sin(angle + hPI) + ")");
			pointRotate(magnitude * Math.signum(RX)); // Set the rotating speed to the magnitude of the left stick
		}


		for (SwerveWheel wheel : wheelArray) { // Apply angles and speeds to swerve wheels
			if (Math.abs(LX) > 0.07 || Math.abs(LY) > 0.07 || Math.abs(RX) > 0.07)
				wheel.goToAngle();
			wheel.drive();
		}
	}

	/**
	 * Calculates the rotation speed needed to correct the robot's rotation
	 * 
	 * @return Rotation speed
	 */
	private double getRotationalCorrection() {
		double currentRotation = Math.toRadians(ahrs.getYaw());
		double angleError = targetRobotAngle - currentRotation;
		if (angleError > Math.PI) {
			angleError -= 2 * Math.PI;
		} else if (angleError < -Math.PI) {
			angleError += 2 * Math.PI;
		}
		// int directionMod = -1;
		// int optimalDirection = 1;
		SmartDashboard.putNumber("Current Target", targetRobotAngle);
		SmartDashboard.putNumber("angleError", angleError);
		double speed = (angleError / Math.PI) * Constants.ROTATIONAL_CORRECTION_FACTOR;

		if (speed > 1)
			speed = 1;
		if (speed < -1)
			speed = -1;

		SmartDashboard.putNumber("Rotational Speed", speed);
		return speed;
	}

	/**
	 * Snaps drive vectors based on controller limiter mode
	 * 
	 * @param angle     The raw input angle
	 * @param magnitude The raw input magnitude
	 * @return The adjusted vectors
	 */
	private double applyControllerLimiter(double angle, double magnitude) {
		double regionFactor = 0;
		double regionNumber = 0;
		switch (controllerMode) {
			case 0: // infinite precision
				SmartDashboard.putString("Control Limiter", "None");
				return angle;

			case 1: // Segmented control
				SmartDashboard.putString("Control Limiter", "Segmented");
				regionFactor = Math.PI / (Constants.numberOfControlSegments / 2);
				return Math.round(angle / regionFactor) * regionFactor;

			case 2: // speed segmented control
				SmartDashboard.putString("Control Limiter", "Speed Segmented");
				if (magnitude < 0.5) {
					regionFactor = Math.PI / (Constants.numberOfControlSegments / 4);
				} else {
					regionFactor = Math.PI / (Constants.numberOfControlSegments / 2);
				}

				return Math.round(angle / regionFactor) * regionFactor;

			case 3: // 4 segmented regions with infinitely precise regions
				SmartDashboard.putString("Control Limiter", "Semi-segmented");
				regionFactor = Math.PI / (Constants.numberOfControlSegments);
				regionNumber = Math.round(angle / regionFactor);

				if (regionNumber == 0 || Math.abs(regionNumber) == Constants.numberOfControlSegments
						|| Math.abs(regionNumber) == Constants.numberOfControlSegments / 2) {
					return regionNumber * regionFactor;
				}
				return angle;

			case 4: // speed segmented control & infinite precision
				SmartDashboard.putString("Control Limiter", "Speed Segmented & Infinite Precision");
				if (magnitude < 0.5) {
					regionFactor = Math.PI / (Constants.numberOfControlSegments / 2);
					return Math.round(angle / regionFactor) * regionFactor;
				} else {
					return angle;
				}

			case 5: // speed segmented control & 4 segmented regions with infinitely precise regions
				SmartDashboard.putString("Control Limiter", "Speed Segmented & Semi-segmented");
				if (magnitude < 0.5) {
					regionFactor = Math.PI / (Constants.numberOfControlSegments / 2);
					return Math.round(angle / regionFactor) * regionFactor;
				} else {
					regionFactor = Math.PI / (Constants.numberOfControlSegments);
					regionNumber = Math.round(angle / regionFactor);

					if (regionNumber == 0 || Math.abs(regionNumber) == Constants.numberOfControlSegments
							|| Math.abs(regionNumber) == Constants.numberOfControlSegments / 2) {
						return regionNumber * regionFactor;
					}
					return angle;
				}

			default:
				return angle;
		}
	}

	/**
	 * Sets controller limiter mode
	 * 
	 * @param mode Vector snapping mode ID
	 */
	public void setControllerLimiter(int mode) {
		controllerMode = (byte) mode;
	}

	/**
	 * Cycles through controller limiter modes
	 */
	public void cycleControllerLimiter() {
		controllerMode++;
		if (controllerMode > 5)
			controllerMode = 0;
	}

	/**
	 * Toggles rotating around a point
	 * @return What the mode is changed to
	 */
	public boolean togglePointRotate() {
		rotateMode = !rotateMode;
		if (rotateMode) {
			calculateRotateAngles(rotatePointX, rotatePointY);
			FRWheel.setTargetAngle(rotateAngles[0]);
			FLWheel.setTargetAngle(rotateAngles[1]);
			BRWheel.setTargetAngle(rotateAngles[2]);
			BLWheel.setTargetAngle(rotateAngles[3]);
		} else {
			FRWheel.setTargetAngle(0);
			FLWheel.setTargetAngle(0);
			BRWheel.setTargetAngle(0);
			BLWheel.setTargetAngle(0);
		}
		System.out.println("Rotate mode: " + rotateMode);
		return rotateMode;
	}

	/**
	 * @return 'true' if the robot is in point rotate mode
	 */
	public boolean isRotating() {
		return rotateMode;
	}

	/**
	 * Set the rotate point of the robot for use in rotate mode
	 * @param X the distance forward (+) to back (-) from the center of the robot (inches)
	 * @param Y the distance right (+) to left (-) from the center of the robot (inches)
	 */
	public void setRotatePoint(double X, double Y) {
		rotatePointX = X;
		rotatePointY = Y;
		calculateRotateAngles(X, Y);
	}

	/**
	 * Caclulates the angles and distances of each of the swerve wheels
	 * @param X the distance forward (+) to back (-) from the center of the robot (inches)
	 * @param Y the distance right (+) to left (-) from the center of the robot (inches)
	 */
	private void calculateRotateAngles(double X, double Y) {
		// System.out.println("X: " + X + " Y: " + Y);
		// Precalculate common values for optimization
		double wMinus = (hWidth  - X);
		double wPlus  = (hWidth  + X);
		double lMinus = (hLength - Y);
		double lPlus  = (hLength + Y);
		double FRAngle;
		double FLAngle;
		double BRAngle;
		double BLAngle;
		// Calculate the angles for each of the wheels, based off of the tangent angle of a circle centered at the point
		if (Y == hLength) { // If the point is between the front wheels, set the front wheels in opposite directions
			FRAngle =  hPI;
			FLAngle = -hPI;
		} else {
			FRAngle = Math.atan2(-wMinus, lMinus);
			FLAngle = Math.atan2( wPlus,  lMinus);
			if (Y < hLength) { // If the point is behind the front wheels, switch the direction of the front wheels
				FRAngle += Math.PI;
				FLAngle += Math.PI;
			}
		}
		if (Y == -hLength) { // If the point is between the back wheels, set the back wheels in opposite directions
			BRAngle =  hPI;
			BLAngle = -hPI;
		} else {
			BRAngle = Math.atan2( wMinus, lPlus);
			BLAngle = Math.atan2(-wPlus,  lPlus);
			if (Y < -hLength) { // If the point is behind the back wheels, switch the direction of the back wheels
				BRAngle += Math.PI;
				BLAngle += Math.PI;
			}
		}
		// Calculate distance from each of the wheels to the point using distance formula
		double FRDist = Math.sqrt((wMinus * wMinus) + (lMinus * lMinus));
		double FLDist = Math.sqrt((wPlus  * wPlus)  + (lMinus * lMinus));
		double BRDist = Math.sqrt((wMinus * wMinus) + (lPlus  * lPlus));
		double BLDist = Math.sqrt((wPlus  * wPlus)  + (lPlus  * lPlus));
		// Find the greatest distance of a wheel to the rotate point
		double maxRotateDist = Math.max(Math.max(FRDist, FLDist), Math.max(BRDist, BLDist));
		// Save calculated values in new arrays
		rotateAngles = new double[] { FRAngle, FLAngle, BRAngle, BLAngle };
		rotateRatios = new double[] { FRDist / maxRotateDist,  FLDist / maxRotateDist,  BRDist / maxRotateDist,  BLDist / maxRotateDist }; // Ratios of the speeds of the wheels compared to the top speed
		FRWheel.setTargetAngle(rotateAngles[0]);
		FLWheel.setTargetAngle(rotateAngles[1]);
		BRWheel.setTargetAngle(rotateAngles[2]);
		BLWheel.setTargetAngle(rotateAngles[3]);
	}

	/* public void setRotatePoint(double distance, double offset) {
		// Precalculate common values for optimization
		double widthMinus = (hWidth - offset);
		double widthPlus  = (hWidth + offset);
		double FRAngle;
		double FLAngle;
		double BRAngle;
		double BLAngle;
		// Calculate the angles for each of the wheels, based off of the tangent angle of a circle centered at the point
		if (distance == 0) { // If the point is between the front wheels, set the front wheels in opposite directions
			FRAngle =  Math.PI / 2;
			FLAngle = -Math.PI / 2;
		} else {
			FRAngle = Math.atan( widthMinus /  distance);
			FLAngle = Math.atan(-widthPlus  /  distance);
			if (distance < 0) { // If the point is behind the front wheels, switch the direction of the front wheels
				FRAngle += Math.PI;
				FLAngle += Math.PI;
			}
		}
		if (distance + Constants.robotLength == 0) { // If the point is between the back wheels, set the back wheels in opposite directions
			BRAngle =  Math.PI / 2;
			BLAngle = -Math.PI / 2;
		} else {
			BRAngle = Math.atan( widthMinus / (distance + Constants.robotLength));
			BLAngle = Math.atan(-widthPlus  / (distance + Constants.robotLength));
			if (distance < -Constants.robotLength) { // If the point is behind the back wheels, switch the direction of the back wheels
				BRAngle += Math.PI;
				BLAngle += Math.PI;
			}
		}
		// Calculate distance from each of the wheels to the point using distance formula
		double FRDist = Math.sqrt((distance * distance) + (widthMinus * widthMinus));
		double FLDist = Math.sqrt((distance * distance) + (widthPlus  * widthPlus));
		double BRDist = Math.sqrt(((distance + Constants.robotLength) * (distance + Constants.robotLength)) + (widthMinus * widthMinus));
		double BLDist = Math.sqrt(((distance + Constants.robotLength) * (distance + Constants.robotLength)) + (widthPlus  * widthPlus));
		// Find the greatest distance of a wheel
		double maxRotateDist = Math.max(Math.max(FRDist, FLDist), Math.max(BRDist, BLDist));
		// Save calculated values in new arrays
		rotateAngles = new double[] { FRAngle, FLAngle, BRAngle, BLAngle };
		rotateRatios  = new double[] { FRDist / maxRotateDist,  FLDist / maxRotateDist,  BRDist / maxRotateDist,  BLDist / maxRotateDist };
		FRWheel.setTargetAngle(rotateAngles[0]);
		FLWheel.setTargetAngle(rotateAngles[1]);
		BRWheel.setTargetAngle(rotateAngles[2]);
		BLWheel.setTargetAngle(rotateAngles[3]);
	} */

	/**
	 * Rotates the robot about the set point
	 * @param RX Speed to rotate around point (-1, 1)
	 */
	public void pointRotate(double speed) {
		speed = - speed * maxTargetSpeed;
		// Sets wheel speeds based on a ratio of their distance to the point
		FRWheel.setSpeed(speed * rotateRatios[0]);
		FLWheel.setSpeed(speed * rotateRatios[1]);
		BRWheel.setSpeed(speed * rotateRatios[2]);
		BLWheel.setSpeed(speed * rotateRatios[3]);
		// Enable wheels
		for (SwerveWheel swerve : wheelArray) {
			swerve.goToAngle();
			swerve.drive();
		}
	}

	public void recalculateWheelPosition() {
		for (SwerveWheel wheel : wheelArray) {
			wheel.resetPosition();
		}
	}

	public void setControlMode(DriveMode mode) {
		switch (mode) {
			case ROBOTCENTRIC:
				isFieldCentric = false;
				isObjectCentric = false;
				frontDirectionOffset = 0;
				break;
			case FIELDCENTRIC:
				isFieldCentric = true;
				isObjectCentric = false;
				break;
			case OBJECTCENTRIC:
				isFieldCentric = false;
				isObjectCentric = true;
				break;
		}
	}

	public DriveMode getControlMode() {
		if (isFieldCentric) {
			return DriveMode.FIELDCENTRIC;
		} else if (isObjectCentric) {
			return DriveMode.OBJECTCENTRIC;
		}
		return DriveMode.ROBOTCENTRIC;
	}

	public void changeFront(Side side) {
		// switch out of field centric
		// set the robot front (N,E,S,W)
		switch (side) {
			case NORTH:
				// isFieldCentric = false;
				isObjectCentric = false;
				frontDirectionOffset = 0;
				break;
			case EAST:
				// isFieldCentric = false;
				isObjectCentric = false;
				frontDirectionOffset = -hPI;
				break;
			case SOUTH:
				// isFieldCentric = false;
				isObjectCentric = false;
				frontDirectionOffset = Math.PI;
				break;
			case WEST:
				// isFieldCentric = false;
				isObjectCentric = false;
				frontDirectionOffset = hPI;
				break;
			default:
				// isFieldCentric = false;
				isObjectCentric = false;
				frontDirectionOffset = 0;
				break;
		}
	}

	public Side getFront() {
		// switch out of field centric
		// set the robot front (N,E,S,W)
		if (frontDirectionOffset == 0) {
			return Side.NORTH;
		} else if (frontDirectionOffset == -hPI) {
			return Side.EAST;
		} else if (frontDirectionOffset == Math.PI) {
			return Side.SOUTH;
		} else if (frontDirectionOffset == hPI) {
			return Side.WEST;
		} else {
			return Side.UNKNOWN;
		}
	}

	public void resetOrentation() {
		ahrs.reset();
		orientationOffset = 0;
		frontDirectionOffset = 0;
		targetRobotAngle = 0;
	}

	public double getDriveSpeed() {
		return maxTargetSpeed;
	}

	public void setDriveSpeed(double speed) {
		speed = Math.abs(speed);
		if (speed > 1)
			speed = 1;
		maxTargetSpeed = speed;
	}

	// public void printPositions() {
	// for (SwerveWheel wheel : wheelArray) {
	// //*System.out.print(wheel.name + "'s position: " + wheel.getRawEncoderValue()
	// + ", ");
	// SmartDashboard.putNumber(wheel.name, wheel.getMotorPosition());
	// }
	// //*System.out.println();
	// }

	private void stopMoving() {
		for (SwerveWheel wheel : wheelArray) {
			wheel.setSpeed(0);
			wheel.drive();
		}
	}

	public void showPositions() {
		for (SwerveWheel wheel : wheelArray) {
			SmartDashboard.putNumber(wheel.getName() + " Position", wheel.rawGetRotation());
			double pos = wheel.rawGetAnalogRotation();
			SmartDashboard.putNumber(wheel.getName() + " Analog Position", pos);
			// SmartDashboard.putNumber(wheel.name + " Analog Raw Position", pos /
			// 0.00080566406);
		}
	}

	public void calibrateHome() {
		for (SwerveWheel wheel : wheelArray) {
			SmartDashboard.putString("calabrating Swerve", wheel.getName());
			wheel.calibBegin();
		}
	}

	public void calibrateMinMax() {
		for (SwerveWheel wheel : wheelArray) {
			// System.out.println("calabrating " + wheel.getName());
			SmartDashboard.putString("calabrating Swerve", wheel.getName());
			wheel.calibFindMinMax();
		}
	}
}
