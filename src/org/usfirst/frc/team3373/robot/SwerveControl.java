package org.usfirst.frc.team3373.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveControl {
	// pushing
	double driveStraightAngle = 0;
	double currentAngle = 0;
	double angleError = 0;
	double p = 5;//5; // 100 is very close
	double i = 0.0005;
	double d = 75;//75; // 125
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

	int robotFront = 1; // Which side of the robot is the front. 1 is default,
						// goes up to 4.

	SwerveWheel LBWheel;
	SwerveWheel LFWheel;
	SwerveWheel RBWheel;
	SwerveWheel RFWheel;

	SwerveWheel[] wheelArray1;
	SwerveWheel[] wheelArray2;

	AHRS ahrs;
	double previousAccelerationX;
	double previousAccelerationY;
	double previousAccelerationZ;

	public SwerveControl(int LBdriveChannel, int LBrotateID, int LBencOffset, int LBEncMin, int LBEncMax,
			double LBWheelMod, int LFdriveChannel, int LFrotateID, int LFencOffset, int LFEncMin, int LFEncMax,
			double LFWheelMod, int RBdriveChannel, int RBrotateID, int RBencOffset, int RBEncMin, int RBEncMax,
			double RBWheelMod, int RFdriveChannel, int RFrotateID, int RFencOffset, int RFEncMin, int RFEncMax,
			double RFWheelMod, double width, double length) {
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

		wheelArray1 = new SwerveWheel[] { LFWheel, RBWheel };
		wheelArray2 = new SwerveWheel[] { LBWheel, RFWheel };
		ahrs = new AHRS(SerialPort.Port.kMXP);
		previousAccelerationX = ahrs.getWorldLinearAccelX();
		previousAccelerationY = ahrs.getWorldLinearAccelY();
		previousAccelerationZ = ahrs.getWorldLinearAccelZ();
		
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

	public void setRobotFront(int side) { // Each side of the robot is assigned
											// a number, this sets which is the
											// active front
		if (side >= 1 && side <= 4) {
			robotFront = side;
		} else {
			robotFront = 1;
		}
	}

	public void calculateSwerveControl(double LY, double LX, double RX) { // MAIN
																			// CALCULATION
																			// METHOD
																			// AND
																			// CONTROL
																			// METHOD!
																			// This
																			// is
																			// what
																			// is
																			// called
																			// in
																			// robot.java,
																			// which
																			// passes
																			// in
																			// the
																			// joystick
																			// values.
		/*
		 * SmartDashboard.putNumber("LY", LY); SmartDashboard.putNumber("LX",
		 * LX); SmartDashboard.putNumber("RX", RX);
		 * SmartDashboard.putNumber("Encoder",
		 * LFWheel.rotateMotor.getAnalogInRaw());
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
			orientationOffset = ahrs.getYaw(); // if in field centric mode make
												// offset equal to the current
												// angle of the navX
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

		translationalAngle += orientationOffset; // sets the robot front to be
													// at the angle determined
													// by orientationOffset
		translationalAngle = translationalAngle % 360;
		if (translationalAngle < 0) {
			translationalAngle += 360;
		}

		translationalYComponent = Math.sin(Math.toRadians(translationalAngle)) * translationalMagnitude; // calculates
																											// y
																											// component
																											// of
																											// translation
																											// vector
		translationalXComponent = Math.cos(Math.toRadians(translationalAngle)) * translationalMagnitude; // calculates
																											// x
																											// component
																											// of
																											// translation
																											// vector

		for (SwerveWheel wheel : wheelArray1) {

			rotateXComponent = Math.cos(Math.toRadians(wheel.getRAngle())) * rotationMagnitude; // calculates
																								// x
																								// component
																								// of
																								// rotation
																								// vector
			rotateYComponent = Math.sin(Math.toRadians(wheel.getRAngle())) * rotationMagnitude; // calculates
																								// y
																								// component
																								// of
																								// rotation
																								// vector

			if (rAxis > 0) {// Why do we do this?
				rotateXComponent = -rotateXComponent;
				rotateYComponent = -rotateYComponent;
			}

			wheel.setSpeed(Math.sqrt(Math.pow(rotateXComponent + translationalXComponent, 2)
					+ Math.pow((rotateYComponent + translationalYComponent), 2)));// sets
																					// the
																					// speed
																					// based
																					// off
																					// translational
																					// and
																					// rotational
																					// vectors
			wheel.setTargetAngle((Math.toDegrees(Math.atan2((rotateYComponent + translationalYComponent),
					(rotateXComponent + translationalXComponent)))));// sets the
																		// target
																		// angle
																		// based
																		// off
																		// translation
																		// and
																		// rotational
																		// vectors
			/*
			 * if(LY == 0 && LX == 0 && RX == 0){//if our inputs are nothing,
			 * don't change the angle(use currentAngle as targetAngle)
			 * wheel.setTargetAngle(wheel.getCurrentAngle()); }
			 */

			if (wheel.getSpeed() > fastestSpeed) {// if speed of wheel is
													// greater than the others
													// store the value
				fastestSpeed = wheel.getSpeed();
			}
		}
		for (SwerveWheel wheel : wheelArray2) {

			rotateXComponent = Math.cos(Math.toRadians(wheel.getRAngle())) * rotationMagnitude; // calculates
																								// x
																								// component
																								// of
																								// rotation
																								// vector
			rotateYComponent = Math.sin(Math.toRadians(wheel.getRAngle())) * rotationMagnitude; // calculates
																								// y
																								// component
																								// of
																								// rotation
																								// vector

			if (rAxis > 0) {// Why do we do this?
				rotateXComponent = -rotateXComponent;
				rotateYComponent = -rotateYComponent;
			}

			wheel.setSpeed(Math.sqrt(Math.pow(rotateXComponent + translationalXComponent, 2)
					+ Math.pow((rotateYComponent + translationalYComponent), 2)));// sets
																					// the
																					// speed
																					// based
																					// off
																					// translational
																					// and
																					// rotational
																					// vectors
			wheel.setTargetAngle((Math.toDegrees(Math.atan2((-rotateYComponent + translationalYComponent),
					(-rotateXComponent + translationalXComponent)))));// sets
																		// the
																		// target
																		// angle
																		// based
																		// off
																		// translation
																		// and
																		// rotational
																		// vectors
			/*
			 * if(LY == 0 && LX == 0 && RX == 0){//if our inputs are nothing,
			 * don't change the angle(use currentAngle as targetAngle)
			 * wheel.setTargetAngle(wheel.getCurrentAngle()); }
			 */

			if (wheel.getSpeed() > fastestSpeed) {// if speed of wheel is
													// greater than the others
													// store the value
				fastestSpeed = wheel.getSpeed();
			}

		}

		if (fastestSpeed > 1) { // if the fastest speed is greater than 1(our
								// max input) divide the target speed for each
								// wheel by the fastest speed
			for (SwerveWheel wheel : wheelArray1) {
				wheel.setSpeed(wheel.getSpeed() / fastestSpeed);
			}
			for (SwerveWheel wheel : wheelArray2) {
				wheel.setSpeed(wheel.getSpeed() / fastestSpeed);
			}
		}

		/*
		 * RFWheel.goToAngle(); LFWheel.goToAngle(); RBWheel.goToAngle();
		 * LBWheel.goToAngle();
		 */
		// if (LY > .05 && LX > .05 && RX > .05) {

		if (LY < .05 && LX < .05 && RX < .05 && LY > -.05 && LX > -.05 && RX > -.05) { // If
																						// the
																						// joystick
																						// is
																						// not
																						// pressed,
																						// keep
																						// the
																						// wheels
																						// where
																						// they
																						// are.
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
	if(inAutonomous){
		if(LBWheel.getAbsClosedLoopError() < 20 && LFWheel.getAbsClosedLoopError() < 20 && RBWheel.getAbsClosedLoopError() < 20 && RFWheel.getAbsClosedLoopError() < 20){
			for(SwerveWheel wheel : wheelArray1){
				wheel.driveWheel();
			}
			for(SwerveWheel wheel : wheelArray2){
				wheel.driveWheel();
			}
		}else{
			for(SwerveWheel wheel : wheelArray1){
				wheel.stopWheel();
			}
			for(SwerveWheel wheel : wheelArray2){
				wheel.stopWheel();
			}
		}
	}else{
		for(SwerveWheel wheel : wheelArray1){
			wheel.driveWheel();
		}
		for(SwerveWheel wheel : wheelArray2){
			wheel.driveWheel();
		}
	}
		
		/*for(SwerveWheel wheel : wheelArray1){
			if(wheel.getAbsClosedLoopError() < 20){
				wheel.driveWheel();
			}
		}
		for(SwerveWheel wheel : wheelArray2){
			if(wheel.getAbsClosedLoopError() < 20){
				wheel.driveWheel();
			}
		}*/
		/*RFWheel.driveWheel(); // Make the wheels drive at their calculated speed
		LFWheel.driveWheel();
		RBWheel.driveWheel();
		LBWheel.driveWheel(); */

	}

	public void calculateObjectControl(double RX) { // Called in robot.java to
													// calculate rotation around
													// an object; Object-centric
													// mode

		radius = objectRadius;
		System.out.println("RADIUSSSSSSSSS!!!: " + radius);

		// TODO change radius to ultrasonic input in order for this to work
		// Radius: distance to front of robot (lengthwise)

		double distanceToFront = radius;
		double distanceToBack = radius + robotLength;

		LFWheel.setTargetAngle(180 + Math.toDegrees(Math.atan2(robotWidth / 2, distanceToFront)));
		RFWheel.setTargetAngle(180 - Math.toDegrees(Math.atan2(robotWidth / 2, distanceToFront)));
		LBWheel.setTargetAngle(180 + Math.toDegrees(Math.atan2(robotWidth / 2, distanceToBack)));
		RBWheel.setTargetAngle(180 - Math.toDegrees(Math.atan2(robotWidth / 2, distanceToBack)));

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
		LFWheel.setSpeedModifier(.2);
		LBWheel.setSpeedModifier(.2);
		RFWheel.setSpeedModifier(.2);
		RBWheel.setSpeedModifier(.2);
	}

	public void normalSpeed() { // Normal speed for all motors
		LFWheel.setSpeedModifier(.5);
		LBWheel.setSpeedModifier(.5);
		RFWheel.setSpeedModifier(.5);
		RBWheel.setSpeedModifier(.5);
	}

	public void setSpinAngle(int angle) {
		spinAngle = angle + ahrs.getAngle();
		isToSpinAngle = false;
	}

	public void spinXdegrees() {
		if (!isToSpinAngle) {
			if (ahrs.getAngle() < spinAngle - 50) {
				calculateSwerveControl(0, 0, .8);
				isToSpinAngle = false;
			} else if (ahrs.getAngle() < spinAngle - .2) {
				calculateSwerveControl(0, 0, .6);
				isToSpinAngle = false;
			} else {
				calculateSwerveControl(0, 0, 0);
				isToSpinAngle = true;
			}
		}
	}

	public void setDriveStraightAngle(double angle) {
		driveStraightAngle = angle;
	}

	public void driveStraight(double speedMod) { // Simple drive straight,
													// correcting around 0
													// degrees. Not the best.
		currentAngle = ahrs.getAngle() + 180;
		currentAngle = currentAngle % 360;
		angleError = (180 + driveStraightAngle) - currentAngle;
		if (angleError > 5) {
			angleError = 5;
		}
		System.out.println("Current Angle:" + currentAngle);
		if (Math.abs(angleError) > .5) {
			if (speedMod < 0) {
				// angleError = angleError *-1;
			}
			calculateSwerveControl(speedMod, 0, 0.03 * angleError);
			System.out.println("Correcting >");
		} else {
			calculateSwerveControl(speedMod, 0, 0);
			System.out.println("Straight");
		}
	}
	public double getXJerk(){
		double deltaAccel = ahrs.getWorldLinearAccelX() -previousAccelerationX;
		previousAccelerationX = ahrs.getWorldLinearAccelX();
		return deltaAccel/.01;
	}
	public double getYJerk(){
		double deltaAccel = ahrs.getWorldLinearAccelY() -previousAccelerationY;
		previousAccelerationY = ahrs.getWorldLinearAccelY();
		return deltaAccel/.01;
	}
	public double getZJerk(){
		double deltaAccel = ahrs.getWorldLinearAccelZ() -previousAccelerationZ;
		previousAccelerationZ = ahrs.getWorldLinearAccelZ();
		return deltaAccel/.01;
	}
	
	
	public void setPID(){
		for (SwerveWheel wheel : wheelArray1) {
			wheel.setPID();
		}
		for (SwerveWheel wheel : wheelArray2) {
			wheel.setPID();
		}
	}
	public void setAutonomousBoolean(boolean auto){
		inAutonomous = auto;
	}
	public void autonomousStraight(){ //Does not hold alignment!!
		isFieldCentric = true;
		calculateSwerveControl(-.8, 0, 0);
	}
	
	public void alignToAngle(int faceAngle){
		if(ahrs.getAngle()%360 > (faceAngle + 5) % 360){
			calculateSwerveControl(0, 0, -.3);
		}else if(ahrs.getAngle()%360 > (faceAngle-5)%360){
			calculateSwerveControl(0, 0, .3);
		}
	}
	
	public void autonomousDrive(int driveAngle, int faceAngle){
		isFieldCentric = true;
		
		double leftXComponent = Math.sin((driveAngle - 90)%360);
		double leftYComponent = Math.cos((driveAngle - 90)%360);
				
		if(ahrs.getAngle()%360 > (faceAngle + 5) % 360){
			calculateSwerveControl(leftXComponent, leftYComponent, 0);
		}else if(ahrs.getAngle()%360 > (faceAngle-5)%360){
			calculateSwerveControl(leftXComponent, leftYComponent, 0);
		}else{
			calculateSwerveControl(leftXComponent, leftYComponent, 0);
		}
	}

}
