package org.first.frc.team3373.autonomous;

import org.usfirst.frc.team3373.robot.DualActuators;
import org.usfirst.frc.team3373.robot.Grabber;
import org.usfirst.frc.team3373.robot.SwerveControl;

public class Auto_1_0 {

	SwerveControl swerve;
	DualActuators lifter;
	Grabber grabber;
	boolean switchLeft;
	boolean scaleLeft;
	boolean hasRisen;
	boolean hasLowered;
	boolean toDistance1R;
	boolean toDistance2R;
	boolean toDistance3R;
	boolean toRotate1R;
	boolean isAtDistance1;
	boolean isAtDistance2;
	boolean placedOnScale;
	boolean hasSpun1;
	boolean gotCube;
	int driveTimer = 0;
	int exportTimer1 = 0;
	int exportTimer2 = 0;
	int shakeCounter = 0;
	int cubeTimer = 0;
	double height;

	public Auto_1_0(SwerveControl swerveDrive, DualActuators actuators, Grabber cubeGrabber, boolean isSwitchLeft,
			boolean isScaleLeft) {
		swerve = swerveDrive;
		lifter = actuators;
		grabber = cubeGrabber;
		switchLeft = isSwitchLeft;
		scaleLeft = isScaleLeft;
		toDistance1R = false;
		toRotate1R = false;
		gotCube = true;
		height = 27.5;
	}


	public void run() {
		shakeCounter++;
		swerve.setAutonomousBoolean(false);
		if (shakeCounter < 40) {
			swerve.calculateSwerveControl(1, 0, 0);
		} else if (shakeCounter < 80) {
			swerve.calculateSwerveControl(-1, 0, 0);
			swerve.resetBump();
		} else if (shakeCounter < 150) {
			swerve.resetBump();
			swerve.resetPositiveX();
			swerve.calculateSwerveControl(0, 0, 0);

		} else if (scaleLeft) {

			this.placeScale();

			if (placedOnScale) {
				if (switchLeft) {

				} else {

				}
			}
		} else {
			if (switchLeft) {

			} else {
				swerve.autonomousDrive(90, 90, .5, .5);
			}

		}
		lifter.goToPosition(height);
	}

	public void placeScale() {
		if (!isAtDistance1) {
			swerve.driveXInchesFromSurface(245, 90, 3);
			if (swerve.isToDistanceFromWall()) {
				isAtDistance1 = true;
				swerve.resetIsToDistance();
			}

		} else if (!isAtDistance2) {
			swerve.driveXInchesFromSurface(60, 90, 1);
			lifter.resetIsToPosition();
			if (swerve.isToDistanceFromWall()) {
				isAtDistance2 = true;
				height = 2;
			}
		} else if (!hasRisen) {
			swerve.calculateSwerveControl(0, 0, 0);
			height = 2;
			if(lifter.isToPosition()) {
				hasRisen = true;
			}
		} else if (exportTimer1 < 50) {
			swerve.calculateSwerveControl(0, 0, 0);
			exportTimer1++;
			grabber.exportCube();
			lifter.resetIsToPosition();
			if(exportTimer1 == 49){
				height = 27.5;
			}
		} else if (!hasLowered) {
			height = 27.5;
			if (lifter.isToPosition()) {
				hasLowered = true;
			}
		} else if (!hasSpun1) {
			swerve.setSpinAngle(270);
			swerve.spinToXdegrees();
			if (swerve.isAtSpinAngle()) {
				hasSpun1 = true;
			}
		}else if(!grabber.hasCube() && cubeTimer < 200){
		swerve.setDriveDistance(75);
		swerve.autonomousDrive(270, 270, .5, .5, 2);
		cubeTimer ++;
		if(cubeTimer == 200){
			gotCube = false;
		}
		}else{
			if(gotCube){
			placedOnScale = true;
			}else{
				swerve.calculateSwerveControl(0, 0, 0);
				grabber.idle();
			}
		}
	}

}
