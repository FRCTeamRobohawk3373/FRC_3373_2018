
package org.usfirst.frc.team3373.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveControl {
	// pushing
	double driveStraightAngle = 0;
	double currentAngle = 0;
	double angleError = 0;
	double p = 5;// 5; // 100 is very close
	double i = 0.0005;
	double d = 75;// 75; // 125
	boolean isRobotCentric = true;
	boolean isFieldCentric = false;
	boolean isObjectCentric = false;
	boolean isHookCentric = false;
	double spinAngle = 0;
	double angleToDiagonal;
	double robotLength;
	double robotWidth;
	double objectRadius = 48;
	
	boolean isToSpinAngle = false;
	
	boolean inAutonomous = false;
	
	double radius;
	
	double orientationOffset;
	double sensorToWheelDistance = 3.25;
	double boilerRadius = 10;
	int startOffset = 0;
	int futureOffset = 0;
	
	int robotFront = 1; // Which side of the robot is the front. 1 is default, goes up to 4.
	
	SwerveWheel LBWheel;
	SwerveWheel LFWheel;
	SwerveWheel RBWheel;
	SwerveWheel RFWheel;
	
	SwerveWheel[] wheelArray1;
	SwerveWheel[] wheelArray2;
	// autonomous sensors
	int integralGainsDriveCounter;
	int spinAngleCounter = 0;
	public UltraSonics ultraSonicSensors;
	public Vision vision;
	// Ultrasonic leftSonic;
	// Ultrasonic rightSonic;
	public AHRS ahrs;
	boolean collidedPositiveY;
	boolean collidedPositiveX;
	boolean collidedNegativeY;
	boolean collidedNegativeX;
	boolean hasBumped;
	double previousAccelerationX;
	double previousAccelerationY;
	double previousAccelerationZ;
	boolean isAtAngle;
	boolean isToPosition;
	double isToPositionCounter;
	double driveDistance;
	int autonomousOffset;
	double previousDistanceReading;
	
	public SwerveControl(int LBdriveChannel, int LBrotateID, int LBencOffset, int LBEncMin, int LBEncMax,
			double LBWheelMod, int LFdriveChannel, int LFrotateID, int LFencOffset, int LFEncMin, int LFEncMax,
			double LFWheelMod, int RBdriveChannel, int RBrotateID, int RBencOffset, int RBEncMin, int RBEncMax,
			double RBWheelMod, int RFdriveChannel, int RFrotateID, int RFencOffset, int RFEncMin, int RFEncMax,
			double RFWheelMod, double width, double length, int leftUltraPort, int rightUltraPort, int backUltraPort) {
		robotWidth = width;
		robotLength = length;
		angleToDiagonal = Math.toDegrees(Math.atan2(length, width));
		LBWheel = new SwerveWheel(LBdriveChannel, LBrotateID, p, i, d, -(270 - angleToDiagonal), 0, LBencOffset,
				LBEncMin, LBEncMax, LBWheelMod);
		LFWheel = new SwerveWheel(LFdriveChannel, LFrotateID, p, i, d, -(angleToDiagonal + 90), 0, LFencOffset,
				LFEncMin, LFEncMax, LFWheelMod);
		RBWheel = new SwerveWheel(RBdriveChannel, RBrotateID, p, i, d, -(angleToDiagonal + 270), 0, RBencOffset,
				RBEncMin, RBEncMax, RBWheelMod);
		RFWheel = new SwerveWheel(RFdriveChannel, RFrotateID, p, i, d, -(90 - angleToDiagonal), 0, RFencOffset,
				RFEncMin, RFEncMax, RFWheelMod);
		
		wheelArray1 = new SwerveWheel[] {LFWheel, RBWheel};
		wheelArray2 = new SwerveWheel[] {LBWheel, RFWheel};
		ahrs = new AHRS(SPI.Port.kMXP);
		vision = new Vision();
		previousAccelerationX = ahrs.getWorldLinearAccelX();
		previousAccelerationY = ahrs.getWorldLinearAccelY();
		previousAccelerationZ = ahrs.getWorldLinearAccelZ();
		collidedPositiveY = false;
		collidedPositiveX = false;
		collidedNegativeY = false;
		collidedNegativeX = false;
		hasBumped = false;
		ultraSonicSensors = new UltraSonics(leftUltraPort, rightUltraPort, backUltraPort);
		isAtAngle = false;
		integralGainsDriveCounter = 0;
		isToPosition = false;
		isToPositionCounter = 0;
		driveDistance = 0;
		autonomousOffset = 0;
		previousDistanceReading = driveDistance;
		
	}
	
	public void turnToAngle(double x, double y) {
		for (SwerveWheel wheel : wheelArray1) {
			wheel.setTargetAngle(LBWheel.calculateTargetAngle(x, y));
			// if(y < .03 &&)
			wheel.rotate();
		}
		for (SwerveWheel wheel : wheelArray2) {
			wheel.setTargetAngle(LBWheel.calculateTargetAngle(x, y));
			// if(y < .03 &&)
			wheel.rotate();
		}
	}
	
	public void setRotateDistance(double distance) {
		objectRadius = distance + boilerRadius + sensorToWheelDistance;
	}
	
	public void setSpeed(double x, double y) {
		if ((Math.abs(x) > .1) || (Math.abs(y) > .1)) {
			LBWheel.drive(x, y);
			LFWheel.drive(x, y);
			RBWheel.drive(x, y);
			RFWheel.drive(x, y);
		} else {
			LBWheel.drive(0, 0);
			LFWheel.drive(0, 0);
			RBWheel.drive(0, 0);
			RFWheel.drive(0, 0);
		}
	}
	
	public void drive(double x, double y) {
		this.turnToAngle(x, y);
		this.setSpeed(x, y);
	}
	
	// Each side of the robot is assigned a number, this sets which is the active front
	public void setRobotFront(int side) {
		if (side >= 1 && side <= 4) {
			robotFront = side;
		} else {
			robotFront = 1;
		}
	}
	
	// MAIN CALCULATION METHOD AND CONTROL METHOD! This is what is called in robot.java, which passes in the joystick
	// values.
	public void calculateSwerveControl(double LY, double LX, double RX) {
		/*
		 * SmartDashboard.putNumber("LY", LY); SmartDashboard.putNumber("LX", LX); SmartDashboard.putNumber("RX", RX);
		 * SmartDashboard.putNumber("Encoder", LFWheel.rotateMotor.getAnalogInRaw());
		 */
		
		double translationalXComponent = LX;
		double translationalYComponent = LY;
		double translationalMagnitude;
		double translationalAngle;
		
		double rAxis = RX;
		double rotateXComponent;
		double rotateYComponent;
		double fastestSpeed = 0;
		
		// Deadband
		if (Math.abs(LX) < 0.1) {
			translationalXComponent = 0;
			LX = 0;
		}
		
		if (Math.abs(LY) < 0.1) {
			translationalYComponent = 0;
			LY = 0;
		}
		
		if (Math.abs(RX) < 0.1) {
			rAxis = 0;
			RX = 0;
		}
		if (isFieldCentric) {
			
			// if in field centric mode make offset equal to the current angle of the navX
			orientationOffset = ahrs.getYaw() - 90 + startOffset + 360;
		} else {
			orientationOffset = 0;
			if (robotFront == 1) {
				orientationOffset += 0;
			} else if (robotFront == 2) {
				orientationOffset += 90;
			} else if (robotFront == 3) {
				orientationOffset += 180;
			} else if (robotFront == 4) {
				orientationOffset += 270;
			}
		}
		
		orientationOffset = orientationOffset % 360;
		
		double rotationMagnitude = Math.abs(rAxis);
		
		translationalMagnitude = Math.sqrt(Math.pow(translationalYComponent, 2) + Math.pow(translationalXComponent, 2));
		translationalAngle = Math.toDegrees(Math.atan2(translationalYComponent, translationalXComponent));
		
		// sets the robot front to be at the angle determined by orientationOffset
		translationalAngle += orientationOffset;
		
		translationalAngle = translationalAngle % 360;
		if (translationalAngle < 0) {
			translationalAngle += 360;
		}
		
		// calculates y component of translation vector
		translationalYComponent = Math.sin(Math.toRadians(translationalAngle)) * translationalMagnitude;
		
		// calculates x component of translation vector
		translationalXComponent = Math.cos(Math.toRadians(translationalAngle)) * translationalMagnitude;
		
		for (SwerveWheel wheel : wheelArray1) {
			
			// calculates x component of rotation vector
			rotateXComponent = Math.cos(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
			
			// calculates y component of rotation vector
			rotateYComponent = Math.sin(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
			
			if (rAxis > 0) {// Why do we do this?
				rotateXComponent = -rotateXComponent;
				rotateYComponent = -rotateYComponent;
			}
			
			// sets the speed based off translational and rotational vectors
			wheel.setSpeed(Math.sqrt(Math.pow(rotateXComponent + translationalXComponent, 2)
					+ Math.pow((rotateYComponent + translationalYComponent), 2)));
			
			// sets the target angle based off translation and rotational vectors
			wheel.setTargetAngle((Math.toDegrees(Math.atan2((rotateYComponent + translationalYComponent),
					(rotateXComponent + translationalXComponent)))));
			
			/*
			 * if(LY == 0 && LX == 0 && RX == 0){//if our inputs are nothing, don't change the angle(use currentAngle as
			 * targetAngle) wheel.setTargetAngle(wheel.getCurrentAngle()); }
			 */
			
			if (wheel.getSpeed() > fastestSpeed) {// if speed of wheel is
													// greater than the others
													// store the value
				fastestSpeed = wheel.getSpeed();
			}
		}
		for (SwerveWheel wheel : wheelArray2) {
			
			// calculates x component of rotation vector
			rotateXComponent = Math.cos(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
			
			// calculates y component of rotation vector
			rotateYComponent = Math.sin(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
			
			if (rAxis > 0) {// Why do we do this?
				rotateXComponent = -rotateXComponent;
				rotateYComponent = -rotateYComponent;
			}
			
			// sets the speed based off translational and rotational vectors
			wheel.setSpeed(Math.sqrt(Math.pow(rotateXComponent + translationalXComponent, 2)
					+ Math.pow((rotateYComponent + translationalYComponent), 2)));
			
			// sets the target angle based off translation and rotational vectors
			wheel.setTargetAngle((Math.toDegrees(Math.atan2((-rotateYComponent + translationalYComponent),
					(-rotateXComponent + translationalXComponent)))));
			/*
			 * if(LY == 0 && LX == 0 && RX == 0){//if our inputs are nothing, don't change the angle(use currentAngle as
			 * targetAngle) wheel.setTargetAngle(wheel.getCurrentAngle()); }
			 */
			
			if (wheel.getSpeed() > fastestSpeed) {// if speed of wheel is greater than the others store the value
				fastestSpeed = wheel.getSpeed();
			}
			
		}
		
		// if the fastest speed is greater than 1(our max input) divide the target speed for each wheel by the fastest
		// speed
		if (fastestSpeed > 1) {
			for (SwerveWheel wheel : wheelArray1) {
				wheel.setSpeed(wheel.getSpeed() / fastestSpeed);
			}
			for (SwerveWheel wheel : wheelArray2) {
				wheel.setSpeed(wheel.getSpeed() / fastestSpeed);
			}
		}
		
		/*
		 * RFWheel.goToAngle(); LFWheel.goToAngle(); RBWheel.goToAngle(); LBWheel.goToAngle();
		 */
		// if (LY > .05 && LX > .05 && RX > .05) {
		
		// If the joystick is not pressed, keep the wheels where they are.
		if (LY < .05 && LX < .05 && RX < .05 && LY > -.05 && LX > -.05 && RX > -.05) {
			RFWheel.setCurrentPosition();
			LFWheel.setCurrentPosition();
			LBWheel.setCurrentPosition();
			RBWheel.setCurrentPosition();
		} else {
			RFWheel.rotate(); // Makes the wheels go to calculated target angle
			LFWheel.rotate();
			LBWheel.rotate();
			RBWheel.rotate();
		}
		if (inAutonomous) {
			if (LBWheel.getAbsClosedLoopError() < 40 && LFWheel.getAbsClosedLoopError() < 40
					&& RBWheel.getAbsClosedLoopError() < 40 && RFWheel.getAbsClosedLoopError() < 40) {
				for (SwerveWheel wheel : wheelArray1) {
					wheel.driveWheel();
				}
				for (SwerveWheel wheel : wheelArray2) {
					wheel.driveWheel();
				}
			} else {
				for (SwerveWheel wheel : wheelArray1) {
					wheel.stopWheel();
				}
				for (SwerveWheel wheel : wheelArray2) {
					wheel.stopWheel();
				}
			}
		} else {
			for (SwerveWheel wheel : wheelArray1) {
				wheel.driveWheel();
			}
			for (SwerveWheel wheel : wheelArray2) {
				wheel.driveWheel();
			}
		}
		
		/*
		 * for(SwerveWheel wheel : wheelArray1){ if(wheel.getAbsClosedLoopError() < 20){ wheel.driveWheel(); } }
		 * for(SwerveWheel wheel : wheelArray2){ if(wheel.getAbsClosedLoopError() < 20){ wheel.driveWheel(); } }
		 */
		/*
		 * RFWheel.driveWheel(); // Make the wheels drive at their calculated speed LFWheel.driveWheel();
		 * RBWheel.driveWheel(); LBWheel.driveWheel();
		 */
		
	}
	
	// Called in robot.java to calculate rotation around an object; Object-centric mode
	public void calculateObjectControl(double RX) {
		
		radius = objectRadius;
		// System.out.println("RADIUSSSSSSSSS!!!: " + radius);
		
		// TODO change radius to ultrasonic input in order for this to work
		// Radius: distance to front of robot (lengthwise)
		
		double distanceToFront = radius;
		double distanceToBack = radius + robotLength;
		
		LFWheel.setTargetAngle(270 + Math.toDegrees(Math.atan2(robotWidth / 2, distanceToFront)));
		RFWheel.setTargetAngle(90 - Math.toDegrees(Math.atan2(robotWidth / 2, distanceToFront)));
		LBWheel.setTargetAngle(270 + Math.toDegrees(Math.atan2(robotWidth / 2, distanceToBack)));
		RBWheel.setTargetAngle(90 - Math.toDegrees(Math.atan2(robotWidth / 2, distanceToBack)));
		
		LBWheel.setSpeed(RX);
		RBWheel.setSpeed(RX);
		
		double speedRatio = Math.sqrt(Math.pow((robotWidth / 2), 2) + Math.pow(distanceToFront, 2))
				/ Math.sqrt(Math.pow((robotWidth / 2), 2) + Math.pow(distanceToBack, 2));
		
		LFWheel.setSpeed(speedRatio * RX);
		RFWheel.setSpeed(speedRatio * RX);
		
		RFWheel.rotate();
		LFWheel.rotate();
		LBWheel.rotate();
		RBWheel.rotate();
		
		// Make the wheels drive at their calculated speed
		RFWheel.driveWheel();
		LFWheel.driveWheel();
		RBWheel.driveWheel();
		LBWheel.driveWheel();
	}
	
	public void turbo() { // Increases speed of all motors
		LFWheel.setSpeedModifier(1);
		LBWheel.setSpeedModifier(1);
		RFWheel.setSpeedModifier(1);
		RBWheel.setSpeedModifier(1);
	}
	
	public void sniper() { // Decreases speed of all motors
		LFWheel.setSpeedModifier(.25);
		LBWheel.setSpeedModifier(.25);
		RFWheel.setSpeedModifier(.25);
		RBWheel.setSpeedModifier(.25);
	}
	
	public void normalSpeed() { // Normal speed for all motors
		LFWheel.setSpeedModifier(.5);
		LBWheel.setSpeedModifier(.5);
		RFWheel.setSpeedModifier(.5);
		RBWheel.setSpeedModifier(.5);
	}
	
	public double getXJerk() {
		double currentAccel = ahrs.getWorldLinearAccelX();
		double deltaAccel = currentAccel - previousAccelerationX;
		previousAccelerationX = currentAccel;
		return deltaAccel / .01;
	}
	
	public double getYJerk() {
		double currentAccel = ahrs.getWorldLinearAccelY();
		double deltaAccel = currentAccel - previousAccelerationY;
		previousAccelerationY = currentAccel;
		return deltaAccel / .01;
	}
	
	public double getZJerk() {
		double currentAccel = ahrs.getWorldLinearAccelZ();
		double deltaAccel = currentAccel - previousAccelerationZ;
		previousAccelerationZ = currentAccel;
		return deltaAccel / .01;
	}
	
	public void setPID() {
		for (SwerveWheel wheel : wheelArray1) {
			wheel.setPID();
		}
		for (SwerveWheel wheel : wheelArray2) {
			wheel.setPID();
		}
	}
	
	public void setAutonomousBoolean(boolean auto) {
		inAutonomous = auto;
	}
	
	public void autonomousStraight() { // Does not hold alignment!!
		isFieldCentric = true;
		calculateSwerveControl(-.8, 0, 0);
	}
	
	public void setSpinAngle(int angle) {
		spinAngle = (angle + 360 - autonomousOffset) % 360;
		// out.println("SpAn: " + spinAngle);
		resetAngleFlag();
	}
	
	public boolean isAtSpinAngle() {
		return isAtAngle;
	}
	
	public void resetAngleFlag() {
		isAtAngle = false;
	}
	
	public void spinToXdegrees() {
		double angleError = (spinAngle - Math.abs((360 - ahrs.getYaw()) % 360));
		double optimalDirection = 1;
		double directionMod = 1;
		if (angleError > 0) {
			directionMod = -1;
		}
		
		if (Math.abs(angleError) > 180) {
			optimalDirection = -1;
			// if(directionMod == -1){
			angleError = (360 - Math.abs(angleError)) % 360;
			// }
		}
		
		this.calculateSwerveControl(0, 0,
				(Math.sqrt(Math.sqrt(Math.abs(angleError))) + 1) * .175 * directionMod * optimalDirection);
		// System.out.println("Spinning.");
		SmartDashboard.putNumber("Anglelelele Error: ", angleError);
		if (Math.abs(angleError) < 4 && spinAngleCounter > 20) { // If the angle is within 4 degrees of the target
			isAtAngle = true;
			spinAngleCounter = 0;
		} else if (Math.abs(angleError) < 4) {
			spinAngleCounter++;
		} else {
			spinAngleCounter = 0;
		}
		SmartDashboard.putNumber("Spincount: ", spinAngleCounter);
		
	}
	
	public void setAutonomousOffset(int offset) {
		autonomousOffset = offset;
	}
	
	public void autonomousDrive(double driveAngle, double faceAngle) {
		isFieldCentric = true;
		faceAngle = (faceAngle + 360 - autonomousOffset) % 360;
		// SmartDashboard.putNumber("face Angle", faceAngle);
		// SmartDashboard.putNumber("Angle Error", angleError);
		// SmartDashboard.putNumber("Angle", (360 - ahrs.getYaw())%360);
		
		double angleError = (faceAngle - Math.abs((360 - ahrs.getYaw()) % 360));
		double optimalDirection = 1;
		double directionMod = 1;
		if (angleError > 0) {
			directionMod = -1;
		}
		
		if (Math.abs(angleError) > 180) {
			optimalDirection = -1;
			// if(directionMod == -1){
			angleError = (360 - Math.abs(angleError)) % 360;
			// }
		}
		
		double leftXComponent = Math.sin(Math.toRadians((driveAngle) % 360));// *XspeedMod;
		double leftYComponent = Math.cos(Math.toRadians((driveAngle) % 360));// *YspeedMod;
		// SmartDashboard.putNumber("Distance Error", distanceError);
		// System.out.println((driveAngle));
		SmartDashboard.putNumber("Angleeel error: ", angleError);
		calculateSwerveControl(leftXComponent, leftYComponent,
				Math.sqrt(Math.sqrt(Math.abs(angleError))) * .1 * directionMod * optimalDirection);
	}
	
	public void autonomousDrive(double driveAngle, double faceAngle, double XspeedMod, double YspeedMod) {
		isFieldCentric = true;
		int directionMod = 1;
		faceAngle = (faceAngle + 360 - autonomousOffset) % 360;
		// SmartDashboard.putNumber("face Angle", faceAngle);
		// SmartDashboard.putNumber("Angle Error", angleError);
		// SmartDashboard.putNumber("Angle", (360 - ahrs.getYaw())%360);
		
		double angleError = (faceAngle - Math.abs((360 - ahrs.getYaw()) % 360));
		double optimalDirection = 1;
		if (angleError > 0) {
			directionMod = -1;
		}
		
		if (Math.abs(angleError) > 180) {
			optimalDirection = -1;
			// if(directionMod == -1){
			angleError = (360 - Math.abs(angleError)) % 360;
			// }
		}
		
		double leftXComponent = Math.sin(Math.toRadians((driveAngle) % 360));// *XspeedMod;
		double leftYComponent = Math.cos(Math.toRadians((driveAngle) % 360));// *YspeedMod;
		// SmartDashboard.putNumber("Distance Error", distanceError);
		// System.out.println((driveAngle));
		// SmartDashboard.putNumber("leftX: ", leftXComponent*XspeedMod);
		// SmartDashboard.putNumber("leftY: ", leftYComponent*YspeedMod);
		// SmartDashboard.putNumber("Angleeel error: ", angleError);
		calculateSwerveControl(leftXComponent * XspeedMod, leftYComponent * YspeedMod,
				Math.sqrt(Math.sqrt(Math.abs(angleError))) * .1 * directionMod * optimalDirection);
	}
	
	public void autonomousDrive(double driveAngle, double faceAngle, double XspeedMod, double YspeedMod,
			int whichUltrasonic) {
		isFieldCentric = true;
		int directionMod = 1;
		faceAngle = (faceAngle + 360 - autonomousOffset) % 360;
		// SmartDashboard.putNumber("face Angle", faceAngle);
		// SmartDashboard.putNumber("Distance",
		// ultraSonicSensors.getDistance(whichUltrasonic));
		// SmartDashboard.putNumber("Angle Error", angleError);
		// SmartDashboard.putNumber("Angle", (360 - ahrs.getYaw())%360);
		
		double angleError = (faceAngle - Math.abs((360 - ahrs.getYaw()) % 360));
		double optimalDirection = 1;
		if (angleError > 0) {
			directionMod = -1;
		}
		
		if (Math.abs(angleError) > 180) {
			optimalDirection = -1;
			// if(directionMod == -1){
			angleError = (360 - Math.abs(angleError)) % 360;
			// }
		}
		double distanceError = driveDistance - Math.cos(angleError) * ultraSonicSensors.getDistance(whichUltrasonic);
		if (whichUltrasonic == 1) {
			if ((driveAngle - faceAngle) % 360 < 180) {
				driveAngle = driveAngle - (distanceError * .0075);
			} else {
				driveAngle = driveAngle + (distanceError * .0075);
			}
		} else if (whichUltrasonic == 2) {
			if ((driveAngle - faceAngle) % 360 < 180) {
				driveAngle = driveAngle + (distanceError * .0075);
			} else {
				driveAngle = driveAngle - (distanceError * .0075);
			}
		} else {
			if ((driveAngle - faceAngle) % 360 <= 270 && (driveAngle - faceAngle) % 360 >= 90) {
				driveAngle = driveAngle + (distanceError * .0075);
			} else {
				driveAngle = driveAngle - (distanceError * .0075);
			}
		}
		double leftXComponent = Math.sin(Math.toRadians((driveAngle) % 360));// *XspeedMod;
		double leftYComponent = Math.cos(Math.toRadians((driveAngle) % 360));// *YspeedMod;
		// SmartDashboard.putNumber("Distance Error", distanceError);
		// System.out.println((driveAngle));
		// SmartDashboard.putNumber("leftX: ", leftXComponent*XspeedMod);
		// SmartDashboard.putNumber("leftY: ", leftYComponent*YspeedMod);
		// SmartDashboard.putNumber("Angleeel error: ", angleError);
		// SmartDashboard.putNumber("Distanceeeeec error: ", distanceError);
		calculateSwerveControl(leftXComponent * XspeedMod, leftYComponent * YspeedMod,
				Math.sqrt(Math.sqrt(Math.abs(angleError))) * .1 * directionMod * optimalDirection);
	}
	
	public void autonomousDriveCrossingOver(double driveAngle, double faceAngle, double XspeedMod, double YspeedMod,
			int whichUltrasonic) {
		double currentDistanceReading = ultraSonicSensors.getDistance(whichUltrasonic);
		if (Math.abs(currentDistanceReading - previousDistanceReading) > 10
				&& Math.abs(currentDistanceReading - previousDistanceReading) < 20) {
			this.setDriveDistance(currentDistanceReading);
		}
		isFieldCentric = true;
		int directionMod = 1;
		faceAngle = (faceAngle + 360 - autonomousOffset) % 360;
		// SmartDashboard.putNumber("face Angle", faceAngle);
		// SmartDashboard.putNumber("Distance",
		// ultraSonicSensors.getDistance(whichUltrasonic));
		// SmartDashboard.putNumber("Angle Error", angleError);
		// SmartDashboard.putNumber("Angle", (360 - ahrs.getYaw())%360);
		
		double angleError = (faceAngle - Math.abs((360 - ahrs.getYaw()) % 360));
		double optimalDirection = 1;
		if (angleError > 0) {
			directionMod = -1;
		}
		
		if (Math.abs(angleError) > 180) {
			optimalDirection = -1;
			// if(directionMod == -1){
			angleError = (360 - Math.abs(angleError)) % 360;
			// }
		}
		double distanceError = driveDistance - Math.cos(angleError) * currentDistanceReading;
		if (whichUltrasonic == 1) {
			if ((driveAngle - faceAngle) % 360 < 180) {
				driveAngle = driveAngle - (distanceError * .0075);
			} else {
				driveAngle = driveAngle + (distanceError * .0075);
			}
		} else if (whichUltrasonic == 2) {
			if ((driveAngle - faceAngle) % 360 < 180) {
				driveAngle = driveAngle + (distanceError * .0075);
			} else {
				driveAngle = driveAngle - (distanceError * .0075);
			}
		} else {
			if ((driveAngle - faceAngle) % 360 <= 270 && (driveAngle - faceAngle) % 360 >= 90) {
				driveAngle = driveAngle + (distanceError * .0075);
			} else {
				driveAngle = driveAngle - (distanceError * .0075);
			}
		}
		double leftXComponent = Math.sin(Math.toRadians((driveAngle) % 360));// *XspeedMod;
		double leftYComponent = Math.cos(Math.toRadians((driveAngle) % 360));// *YspeedMod;
		// SmartDashboard.putNumber("Distance Error", distanceError);
		// System.out.println((driveAngle));
		// SmartDashboard.putNumber("leftX: ", leftXComponent*XspeedMod);
		// SmartDashboard.putNumber("leftY: ", leftYComponent*YspeedMod);
		// SmartDashboard.putNumber("Angleeel error: ", angleError);
		// SmartDashboard.putNumber("Distanceeeeec error: ", distanceError);
		calculateSwerveControl(leftXComponent * XspeedMod, leftYComponent * YspeedMod,
				Math.sqrt(Math.sqrt(Math.abs(angleError))) * .1 * directionMod * optimalDirection);
		previousDistanceReading = currentDistanceReading;
		
	}
	
	public void driveToTape(double driveAngle, double faceAngle, int whichCamera) {
		double tapeOffset = 0;
		vision.switchCamera(whichCamera);
		VisionObject visionObject = vision.getClosestObject(3);
		if (!(visionObject == null)) {
			tapeOffset = (visionObject.X * -1) * 10;
		} else {
			tapeOffset = 0;
		}
		autonomousDrive(driveAngle + tapeOffset, faceAngle);
	}
	
	public void setDriveDistance(double distance) {
		driveDistance = distance;
	}
	
	public void driveXInchesFromSurface(double target, int faceAngle, int whichUltrasonic) {
		double deltaDistance = (target - ultraSonicSensors.getDistance(whichUltrasonic));
		double direction = deltaDistance / Math.abs(deltaDistance);
		double motorPower = Math.sqrt((Math.abs(deltaDistance))) * .15;
		if (motorPower > 1) {
			motorPower = 1;
		}
		if (motorPower < -1) {
			motorPower = -1;
		}
		if (Math.abs(deltaDistance) > 150) {
			turbo();
		} else {
			normalSpeed();
		}
		if (whichUltrasonic == 1) {
			if (direction > 0) this.autonomousDrive(faceAngle + 270, faceAngle, motorPower, motorPower);
			else this.autonomousDrive(faceAngle + 90, faceAngle, motorPower, motorPower);
		} else if (whichUltrasonic == 2) {
			if (direction > 0) this.autonomousDrive(faceAngle + 90, faceAngle, motorPower, motorPower);
			else this.autonomousDrive(faceAngle + 270, faceAngle, motorPower, motorPower);
		} else {
			if (direction > 0) this.autonomousDrive(faceAngle, faceAngle, motorPower, motorPower);
			else this.autonomousDrive(faceAngle + 180, faceAngle, motorPower, motorPower);
		}
		if (Math.abs(deltaDistance) < 3) isToPositionCounter++;
		else {
			isToPositionCounter = 0;
		}
		if (isToPositionCounter > 3) isToPosition = true;
	}
	
	public void driveXInchesFromSurface(double target, int faceAngle, int directionalUltrasonic, boolean crossingCubes,
			int wallUltrasonic) {
		double deltaDistance = (target - ultraSonicSensors.getDistance(directionalUltrasonic));
		double direction = deltaDistance / Math.abs(deltaDistance);
		double motorPower = Math.sqrt((Math.abs(deltaDistance))) * .15;
		if (motorPower > 1) {
			motorPower = 1;
		}
		if (motorPower < -1) {
			motorPower = -1;
		}
		if (Math.abs(deltaDistance) > 150) {
			turbo();
		} else {
			normalSpeed();
		}
		if (!crossingCubes) {
			if (directionalUltrasonic == 1) {
				if (direction > 0)
					this.autonomousDrive(faceAngle + 270, faceAngle, motorPower, motorPower, wallUltrasonic);
				else this.autonomousDrive(faceAngle + 90, faceAngle, motorPower, motorPower, wallUltrasonic);
			} else if (directionalUltrasonic == 2) {
				if (direction > 0)
					this.autonomousDrive(faceAngle + 90, faceAngle, motorPower, motorPower, wallUltrasonic);
				else this.autonomousDrive(faceAngle + 270, faceAngle, motorPower, motorPower, wallUltrasonic);
			} else {
				if (direction > 0) this.autonomousDrive(faceAngle, faceAngle, motorPower, motorPower, wallUltrasonic);
				else this.autonomousDrive(faceAngle + 180, faceAngle, motorPower, motorPower, wallUltrasonic);
			}
		} else {
			if (directionalUltrasonic == 1) {
				if (direction > 0) this.autonomousDriveCrossingOver(faceAngle + 270, faceAngle, motorPower, motorPower,
						wallUltrasonic);
				else this.autonomousDriveCrossingOver(faceAngle + 90, faceAngle, motorPower, motorPower,
						wallUltrasonic);
			} else if (directionalUltrasonic == 2) {
				if (direction < 0)
					this.autonomousDriveCrossingOver(faceAngle + 90, faceAngle, motorPower, motorPower, wallUltrasonic);
				else this.autonomousDriveCrossingOver(faceAngle + 270, faceAngle, motorPower, motorPower,
						wallUltrasonic);
			} else {
				if (direction > 0)
					this.autonomousDriveCrossingOver(faceAngle, faceAngle, motorPower, motorPower, wallUltrasonic);
				else this.autonomousDriveCrossingOver(faceAngle + 180, faceAngle, motorPower, motorPower,
						wallUltrasonic);
			}
		}
		SmartDashboard.putNumber("Pos Counter: ", isToPositionCounter);
		SmartDashboard.putNumber("Right Ultrasonic DIS", ultraSonicSensors.getRawDistance(2));
		if (Math.abs(deltaDistance) < 3) isToPositionCounter++;
		else {
			isToPositionCounter = 0;
		}
		if (isToPositionCounter > 3) isToPosition = true;
	}
	
	public boolean isToDistanceFromWall() {
		return isToPosition;
	}
	
	public void resetIsToDistance() {
		isToPosition = false;
		isToPositionCounter = 0;
	}
	
	public boolean hasCollidedPositiveX() {
		if (this.getXJerk() > 100) collidedPositiveX = true;
		return collidedPositiveX;
	}
	
	public boolean hasCollidedPositiveY() {
		if (this.getYJerk() > 100) collidedPositiveY = true;
		return collidedPositiveY;
	}
	
	public boolean hasCollidedNegativeX() {
		if (this.getXJerk() < -100) collidedNegativeX = true;
		return collidedNegativeX;
	}
	
	public boolean hasCollidedNegativeY() {
		if (this.getXJerk() < -100) collidedNegativeY = true;
		return collidedNegativeY;
	}
	
	public boolean hasHitBump() {
		if (Math.abs(this.getZJerk()) > 250) hasBumped = true;
		return hasBumped;
	}
	
	public void resetBump() {
		hasBumped = false;
	}
	
	public void resetNegativeY() {
		collidedNegativeY = false;
	}
	
	public void resetNegativeX() {
		collidedNegativeX = false;
	}
	
	public void resetPositiveX() {
		collidedPositiveX = false;
	}
	
	public void resetPositiveY() {
		collidedPositiveY = false;
	}
	
	public void setStartOffset(int offset) {
		futureOffset = offset * -1;
	}
	
	public void activateStartOffset() {
		startOffset = futureOffset;
	}
	
	public void activateAutoOffset() {
		startOffset = autonomousOffset;
	}
}
