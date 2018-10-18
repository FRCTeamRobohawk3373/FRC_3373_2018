package org.usfirst.frc.team3373.robot;

import com.ctre.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveWheel {
	
	WPI_TalonSRX rotateMotor; // The motor which spins the assembly
	WPI_TalonSRX driveMotor; // The motor powering the wheel
	private double previousEncValue;
	boolean increasing;
	private double targetAngle;
	private double speed;
	private int encoderUnitsPerRotation;// Range of encoder
	private double speedModifier = .5;
	private int encOffset; // The value of the abs. pos. encoder at home
							// position (Bolt hole facing front)
	double rotateAngle; // The offset for each wheel to rotate based on which
						// corner it is in
	double wheelModifier; // The multiplier to account for varying wheel speeds
	
	int rotationEncoderMin;
	int rotationEncoderMax;
	double P;
	double I;
	double D;
	
	public SwerveWheel(int driveMotorChannel, int rotateMotorID, double p, double i, double d, double rotateAng,
			int distanceFromZero, int encoderOffset, int encoderMin, int encoderMax, double wheelMod) {
		P = p;
		I = i;
		D = d;
		previousEncValue = 50;
		increasing = true;
		rotateMotor = new WPI_TalonSRX(rotateMotorID);
		
		// Read the analog input from the abs. pos. encoder
		rotateMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 1);
		/*
		 * rotateMotor.changeControlMode(position); Not actual line, this is a reminder!
		 */
		rotateMotor.setSensorPhase(false);
		
		// Ensures that the overflow is unused
		rotateMotor.setSelectedSensorPosition(rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		// setPID();
		driveMotor = new WPI_TalonSRX(driveMotorChannel);
		
		encOffset = encoderOffset;
		rotationEncoderMin = encoderMin;
		rotationEncoderMax = encoderMax;
		encoderUnitsPerRotation = encoderMax - encoderMin;
		
		// rotateMotor.setPID(p, i, d);
		
		// Configures the closed loop motor control
		
		rotateMotor.overrideLimitSwitchesEnable(false);
		rotateMotor.setNeutralMode(NeutralMode.Brake); // Activates brake mode
		driveMotor.setNeutralMode(NeutralMode.Brake); // Activates brake mode
		rotateAngle = rotateAng;
		wheelModifier = wheelMod;
		setPID();
		
	}
	
	public int calculateTargetAngle(double x, double y) { // Used in turnToAngle
		int angle = (int) Math.toDegrees(Math.atan2(y, -x));
		if (angle <= 0) {
			angle += 360;
		}
		return angle;
		
	}
	
	// Actually used! This method calculates the target encoder position, fed directly into the set() method in
	// rotate(), accounting for offset as a result of the encoder zero value.
	public int calculateTargetEncoder() {
		double targetEnc = targetAngle;
		targetEnc = targetEnc / 360;
		targetEnc = targetEnc * encoderUnitsPerRotation;
		targetEnc += encOffset;
		targetEnc = targetEnc % rotationEncoderMax;
		if (targetEnc < rotationEncoderMin) {
			targetEnc = rotationEncoderMin;
		}
		int finalityal = (int) targetEnc;
		return finalityal;
	}
	
	public void setTargetAngle(double TA) {
		targetAngle = TA;
	}
	
	public double getTargetAngle() {
		return targetAngle;
	}
	
	public void setSpeed(double speedIn) {
		speed = speedIn * wheelModifier;
	}
	
	public double getSpeed() {
		return speed;
	}
	
	public int angleToEncoderUnit(double angle) { // Math to convert angles to encoder units
		
		double deltaEncoder;
		deltaEncoder = angle * (encoderUnitsPerRotation / 360.0);
		
		return (int) deltaEncoder;
	}
	
	public int encoderUnitToAngle(int encoderValue) { // Math to convert encoder unis to angle
		double angle = 0;
		if (encoderValue >= 0) {
			angle = (encoderValue * (360.0 / encoderUnitsPerRotation));
			angle = angle % 360;
		} else if (encoderValue < 0) {
			angle = (encoderValue * (360.0 / encoderUnitsPerRotation));
			angle = angle % 360 + 360;
		}
		return (int) angle;
	}
	
	public void setSpeedModifier(double speed) {
		speedModifier = speed;
	}
	
	public void disable() {
		rotateMotor.set(0);
	}
	
	public double getCurrentAngle() {
		double currentAngle = (rotateMotor.getSensorCollection().getAnalogInRaw() - encOffset);
		if (currentAngle <= 0) {
			currentAngle += rotationEncoderMax;
		}
		currentAngle = currentAngle / (encoderUnitsPerRotation / 360.0);
		return currentAngle;
	}
	
	public double calculateDriveSpeed(double x, double y) {
		double speed = 0;
		double square = (x * x) + (y * y);
		speed = Math.sqrt(square);
		return speed;
	}
	
	public void drive(double x, double y) { // Activates the drive motor
		/*
		 * if(Math.abs(targetAngle-currentAngle) > 2)
		 * driveMotor.set(speed*speedModifier); else{
		 * driveMotor.set(-speed*speedModifier); }
		 */
		SmartDashboard.putNumber("Speed: " + this.toString(), speed * speedModifier * wheelModifier);
		driveMotor.set(calculateDriveSpeed(x, y) * speedModifier * wheelModifier);// *directionalModifier
	}
	
	public void driveWheel() { // Activates drive motor
		driveMotor.set(speed * speedModifier);// *directionalModifier
	}
	
	public void stopWheel() {
		driveMotor.set(0);
	}
	
	public int encoderCheck(int targetEncoder) { // Ensures encoders are not
													// trying to go out of range
		int target = 0;
		target = targetEncoder;
		// target += encOffset;
		target += rotationEncoderMax;
		target = target % rotationEncoderMax;
		if (target < rotationEncoderMin) {
			target = rotationEncoderMin;
		}
		return target;
	}
	
	public void rotate() { // Activates the rotation motor
		// rotateMotor.changeControlMode(TalonControlMode.Position);
		setPID();
		int target = angleToEncoderUnit(getDeltaTheta()) + rotateMotor.getSensorCollection().getAnalogInRaw();
		target = encoderCheck(target);
		rotateMotor.set(ControlMode.Position, target);
	}
	
	public int getEncoderTarget() {
		int encoderTarget = angleToEncoderUnit(getDeltaTheta()) + rotateMotor.getSensorCollection().getAnalogInRaw();
		return encoderTarget;
	}
	
	public double getDeltaTheta() { // Finds the change in angle
		double deltaTheta = getTargetAngle() - getCurrentAngle();
		
		while ((deltaTheta < -90) || (deltaTheta > 90)) {
			if (deltaTheta > 90) {
				deltaTheta -= 180;
				speed *= -1;
			} else if (deltaTheta < -90) {
				deltaTheta += 180;
				speed *= -1;
			}
		}
		
		// if (deltaTheta >= 1 || deltaTheta <= -1){
		return deltaTheta;
		// } else {
		// return 0;
		// }
		
	}
	
	public double getRAngle() {
		return rotateAngle;
	}
	
	public void setCurrentPosition() {
		rotateMotor.set(ControlMode.Position, rotateMotor.getSensorCollection().getAnalogInRaw());
	}
	
	public void changePID(double p, double i, double d) {
		P = p;
		I = i;
		D = d;
	}
	
	public void setPID() {
		// rotateMotor.config_IntegralZone(0, 150, 0);
		// if(Math.abs(angleToEncoderUnit(getDeltaTheta())) > 50){
		/*
		 * rotateMotor.config_kP(5, 0, 0);
		 * rotateMotor.config_kI(0, 0, 0);
		 * rotateMotor.config_kD(75, 0, 0);
		 */
		
		if (Math.abs(rotateMotor.getClosedLoopError(0)) > 20) {
			rotateMotor.config_kP(0, P + (20 / Math.pow((Math.abs((rotateMotor.getClosedLoopError(0))) + 1), 7)), 0);
			rotateMotor.config_kI(0, I, 0);
			rotateMotor.config_kD(0, D, 0);
		} else {
			rotateMotor.config_kP(0, P/* +4.5/4 */, 0);
			rotateMotor.config_kI(0, I, 0);
			rotateMotor.config_kD(0, D, 0);
		}
		
		/*
		 * }else{
		 * rotateMotor.config_kP(0, 3.25,0);// +(Math.pow((Math.abs((angleToEncoderUnit(getDeltaTheta()))) + 3),1/2)),
		 * 0);
		 * rotateMotor.config_kI(0, .00325*Math.abs(rotateMotor.getOutputCurrent()), 0);
		 * rotateMotor.config_kD(0, 0, 0);
		 * }
		 */
		/*
		 * rotateMotor.config_kP(0, 5, 0);
		 * rotateMotor.config_kI(0, 0, 0);
		 * rotateMotor.config_kD(0, 75, 0);
		 */
		/*
		 * if(Math.abs(getEncoderTarget() - rotateMotor.getSensorCollection().getAnalogInRaw()) > 75){
		 * rotateMotor.config_kP(0, SmartDashboard.getNumber("P",0), 0);
		 * rotateMotor.config_kI(0, SmartDashboard.getNumber("I",0), 0);
		 * rotateMotor.config_kD(0, SmartDashboard.getNumber("D", 0), 0);
		 * }else{
		 * rotateMotor.config_kP(0, SmartDashboard.getNumber("P",0) * 5, 0);
		 * rotateMotor.config_kI(0, SmartDashboard.getNumber("I",0), 0);
		 * rotateMotor.config_kD(0, SmartDashboard.getNumber("D", 0), 0);
		 * }
		 */
		/*
		 * rotateMotor.config_kP(0, P, 0);
		 * rotateMotor.config_kI(0, I, 0);
		 * rotateMotor.config_kD(0, D, 0);
		 */
	}
	
	public boolean getPeriod() {
		double currentEncValue = rotateMotor.getSensorCollection().getAnalogInRaw();
		if (currentEncValue < previousEncValue - 2 && increasing) {
			return true;
		} else if (currentEncValue > previousEncValue + 2 && !increasing) {
			return true;
		} else if (currentEncValue > previousEncValue + 2) {
			increasing = true;
		} else if (currentEncValue < previousEncValue - 2) {
			increasing = false;
		}
		previousEncValue = currentEncValue;
		return false;
	}
	
	public void goToEncPosition(double encoder) {
		double output = 0;
		double error = encoder - rotateMotor.getSensorCollection().getAnalogInRaw();
		
	}
	
	public int getAbsClosedLoopError() {
		return Math.abs(rotateMotor.getClosedLoopError(0));
	}
}
