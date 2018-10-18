/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3373.robot;

import org.first.frc.team3373.autonomous.*;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	
	
	AutonomousControl autoController;
	
	int robotCounter = 0;
	
	
	
	
	//Swerve Initialization
	SwerveControl swerve;
	
	int FLEncoderCalibMin = 300;  //Calibration defaults, which update as calibration is done.
	int FLEncoderCalibMax = 300;
	int FREncoderCalibMin = 300;
	int FREncoderCalibMax = 300;
	int BLEncoderCalibMin = 300;
	int BLEncoderCalibMax = 300;
	int BREncoderCalibMin = 300;
	int BREncoderCalibMax = 300;
	
	double robotWidth = 22.75; //TODO change robot dimensions to match this years robot
	double robotLength = 27.375;
	//double robotWidth = 20.75; //TODO change robot dimensions to match this years robot
	//double robotLength = 26.8125;

/*
	int LBdriveChannel = 1;
	int LBrotateID = 2;
	int LBencOffset = 420; // Zero values (value when wheel is turned to default
							// zero- bolt hole facing front.)
	int LBEncMin = 10;
	int LBEncMax = 880;
	double LBWheelMod = .8922; //Modifier for wheel speed

	int LFdriveChannel = 4;
	int LFrotateID = 3;
	int LFencOffset = 604;
	int LFEncMin = 12;
	int LFEncMax = 899;
	double LFWheelMod = .9479;

	int RBdriveChannel = 8;
	int RBrotateID = 7;
	int RBencOffset = 486;
	int RBEncMin = 11;
	int RBEncMax = 900;
	double RBWheelMod = .95;

	int RFdriveChannel = 6;
	int RFrotateID = 5;
	int RFencOffset = 212;
	int RFEncMin = 11;
	int RFEncMax = 895;
	double RFWheelMod = .8922;
	*/
	
	int LBdriveChannel = 2;
	int LBrotateID = 1;
	int LBencOffset = 590; // Zero values (value when wheel is turned to default
							// zero- bolt hole facing front.)
	int LBEncMin = 10;
	int LBEncMax = 879;
	double LBWheelMod = 1; //Modifier for wheel speed

	int LFdriveChannel = 4;
	int LFrotateID = 3;
	int LFencOffset = 602;
	int LFEncMin = 11;
	int LFEncMax = 889;
	double LFWheelMod = 1;

	int RBdriveChannel = 8;
	int RBrotateID = 7;
	int RBencOffset = 317;
	int RBEncMin = 12;
	int RBEncMax = 885;
	double RBWheelMod = 1;

	int RFdriveChannel = 6;
	int RFrotateID = 5;
	int RFencOffset = 65;
	int RFEncMin = 9;
	int RFEncMax = 891;
	double RFWheelMod = 1;
	
	int leftUltraSonicPort =2;
	int rightUltraSonicPort = 1;
	int backUltraSonicPort = 0; //Timber
	
	//lexi
	/*int leftUltraSonicPort =1;
	int rightUltraSonicPort = 0;
	int backUltraSonicPort = 2;
	*/
	//Dual Linear Actuator Configs
	//Look at Actuator.calibrate to view documentaion about how to calculate individual Actuators
	static int actuator1Port1 = 9; //left
	static int actuator1Port2 = 10;//right
	static int actuator2Port1 = 11;
	static int actuator2Port2 = 12;
	static double minPot1 = 76;
	static double minPot2 = 143;
	static double maxPot1 = 731;
	static double maxPot2 = 802;
	static double minDistance1 = 2;
	static double minDistance2 = 2;
	static double maxDistance1 = 27.5;
	static double maxDistance2 = 27.5;
	
	DigitalInput positionalOnes; // Input for the 16-slot dial
	DigitalInput positionalTwos;
	DigitalInput positionalFours;
	
	DigitalInput programOnes; // Input for the 16-slot dial
	DigitalInput programTwos;
	DigitalInput programFours;
	
	//Grabber Initialization
	int grabberPort1 =13; // Need to updtate for the Robot
	int grabberPort2 =14;
	Grabber grabber;
	
	
	
	//Values for SuperJoystick getRawAxis()
	int LX = 0;
	int LY = 1;
	int Ltrigger = 2;
	int Rtrigger = 3;
	int RX = 4;
	int RY = 5;
	
	Actuator act1;
	
	DualActuators lifter;
	
	SuperJoystick driver;
	SuperJoystick shooter;
	
	ClimbBar climbBar;
	
	int positionalIndex = 1;// for testing purposes
	int programIndex = 1;// for testing purposes
	
	double xJerkMax;
	double yJerkMax;
	double zJerkMax;
	
	double xAccelMax;
	double yAccelMax;
	double zAccelMax;
	
//	Vision vision;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
//		vision = new Vision();
		robotCounter = 0;
		
		positionalOnes = new DigitalInput(3);
		positionalTwos = new DigitalInput(2);
		positionalFours = new DigitalInput(1);
		
		programOnes = new DigitalInput(7);
		programTwos = new DigitalInput(6);
		programFours = new DigitalInput(5);
		
		positionalIndex = 7;
		
		if (positionalOnes.get()) {
			positionalIndex -= 1;
		}
		if (positionalTwos.get()) {
			positionalIndex -= 2;
		}
		if (positionalFours.get()) {
			positionalIndex -= 4;
		}
		if (positionalIndex == 8) {
			positionalIndex = 0;
		}
		positionalIndex += 8;
		positionalIndex%=8;
		programIndex = 6;

		if (programOnes.get()) {
			programIndex -= 1;
		}
		if (programTwos.get()) {
			programIndex -= 2;
		}
		if (programFours.get()) {
			programIndex -= 4;
		}
		if (programIndex == 8) {
			programIndex = 0;
		}
		programIndex += 8;
		programIndex %= 8;



		lifter = new DualActuators(actuator1Port1,actuator2Port1,actuator1Port2,actuator2Port2,maxPot1,maxPot2,minPot1,minPot2,maxDistance1,maxDistance2,minDistance1,minDistance2);
		swerve = new SwerveControl(LBdriveChannel, LBrotateID, LBencOffset, LBEncMin, LBEncMax, LBWheelMod, LFdriveChannel,
				LFrotateID, LFencOffset, LFEncMin, LFEncMax, LFWheelMod, RBdriveChannel, RBrotateID, RBencOffset, RBEncMin,
				RBEncMax, RBWheelMod, RFdriveChannel, RFrotateID, RFencOffset, RFEncMin, RFEncMax, RFWheelMod, robotWidth, robotLength, leftUltraSonicPort, rightUltraSonicPort, backUltraSonicPort);
		
		
		this.setPeriod(.01);
				
		swerve.LBWheel.rotateMotor.setSelectedSensorPosition(swerve.LBWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.RBWheel.rotateMotor.setSelectedSensorPosition(swerve.RBWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.LFWheel.rotateMotor.setSelectedSensorPosition(swerve.LFWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.RFWheel.rotateMotor.setSelectedSensorPosition(swerve.RFWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		
		driver = new SuperJoystick(0);
		shooter = new SuperJoystick(1);
		
		grabber = new Grabber(grabberPort1,grabberPort2);
		climbBar = new ClimbBar(2);
		
		xJerkMax = 0;
		yJerkMax = 0;
		zJerkMax = 0;
		
		xAccelMax = 0;
		yAccelMax = 0;
		zAccelMax = 0;
	}

	/**
	 * Up is negative, Down is positive
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		//vision.switchCamera(0);
		autoController = new AutonomousControl(positionalIndex, programIndex, swerve, lifter, grabber);
		swerve.ahrs.reset();
	//	swerve.setDriveDistance(swerve.ultraSonicSensors.getDistance(1));

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	public void autonomousPeriodic() {
		//swerve.autonomousDrive(0, 90, 1, 1, 3);
		swerve.setAutonomousBoolean(true);
		SmartDashboard.putNumber("Back Ultraboi!", swerve.ultraSonicSensors.getDistance(3));
		//swerve.driveForwardXInchesFromSurface(50, 90);
		//SmartDashboard.putNumber("Angle", swerve.ahrs.getYaw());
		//swerve.driveForwardXInchesFromSurface(100, 97);
		/*SmartDashboard.putNumber("Voltage3", swerve.ultraSonicSensors.getVoltage(3));
		SmartDashboard.putNumber("Voltage2", swerve.ultraSonicSensors.getVoltage(2));
		SmartDashboard.putNumber("Voltage1", swerve.ultraSonicSensors.getVoltage(1));
		SmartDashboard.putNumber("Distance1", swerve.ultraSonicSensors.getDistance(1));
		SmartDashboard.putNumber("Distance2", swerve.ultraSonicSensors.getDistance(2));
		SmartDashboard.putNumber("Distance3", swerve.ultraSonicSensors.getDistance(3));
		//SmartDashboard.putNumber("Voltage2", swerve.leftSonic.getVoltage());
		//SmartDashboard.putNumber("Voltage3", swerve.rightSonic.getVoltage());
		//SmartDashboard.putNumber("Distance", swerve.backSonic.getDistance());
		*/
		SmartDashboard.putNumber("Back Ultraboi!", swerve.ultraSonicSensors.getDistance(3));

		swerve.LBWheel.rotateMotor.setSelectedSensorPosition(swerve.LBWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.RBWheel.rotateMotor.setSelectedSensorPosition(swerve.RBWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.LFWheel.rotateMotor.setSelectedSensorPosition(swerve.LFWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.RFWheel.rotateMotor.setSelectedSensorPosition(swerve.RFWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		//swerve.driveXInchesFromSurface(21.5, 0, 3);
		autoController.activateAuto();
	}

	
	
	public void teleopInit(){
		swerve.activateStartOffset();
	}
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
	//DriverStation.getInstance().
		SmartDashboard.putNumber("Back Ultrasonic", swerve.ultraSonicSensors.getDistance(3));
		SmartDashboard.putNumber("Left Ultrasonic", swerve.ultraSonicSensors.getDistance(1));
		SmartDashboard.putNumber("Right Ultrasonic", swerve.ultraSonicSensors.getDistance(2));
		swerve.setAutonomousBoolean(false);
		this.activateControl();

		
		swerve.LBWheel.rotateMotor.setSelectedSensorPosition(swerve.LBWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.RBWheel.rotateMotor.setSelectedSensorPosition(swerve.RBWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.LFWheel.rotateMotor.setSelectedSensorPosition(swerve.LFWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.RFWheel.rotateMotor.setSelectedSensorPosition(swerve.RFWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		
		
		
		//Lift Code for Actuators Moving Together
		//SmartDashboard.putNumber("Current", lifter.getOutputCurrent());
		
		
		//Grabber Code
		/*if(shooter.isLBHeld())
			grabber.exportCube();
		else if(shooter.isRBHeld())
			grabber.importCube();
		else if(grabber.hasCube())
			grabber.keepCube();
		else
			grabber.idle();
			*/
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	//	lifter.calibrate(shooter, false);
		lifter.calibrate(shooter, false);
		//lifter.calibrate(shooter, false);
		SmartDashboard.putNumber("Actuator Position", lifter.getPosition());
		//lifter.calibrate(shooter, false);
		/*actuators.calibrate(shooter, false);
		actuators.calibrate(shooter, true); */
		
	}
	
	public void calibrateSwerve(){
		
		swerve.RFWheel.rotateMotor.set(ControlMode.Disabled, 0);
		swerve.RBWheel.rotateMotor.set(ControlMode.Disabled, 0);
		swerve.LFWheel.rotateMotor.set(ControlMode.Disabled, 0);
		swerve.LBWheel.rotateMotor.set(ControlMode.Disabled, 0);

		if (swerve.RFWheel.rotateMotor.getSensorCollection().getAnalogInRaw() < FREncoderCalibMin) {
			FREncoderCalibMin = swerve.RFWheel.rotateMotor.getSensorCollection().getAnalogInRaw();
		}
		if (swerve.RFWheel.rotateMotor.getSensorCollection().getAnalogInRaw() > FREncoderCalibMax) {
			FREncoderCalibMax = swerve.RFWheel.rotateMotor.getSensorCollection().getAnalogInRaw();
		}

		if (swerve.RBWheel.rotateMotor.getSensorCollection().getAnalogInRaw() < BREncoderCalibMin) {
			BREncoderCalibMin = swerve.RBWheel.rotateMotor.getSensorCollection().getAnalogInRaw();
		}
		if (swerve.RBWheel.rotateMotor.getSensorCollection().getAnalogInRaw() > BREncoderCalibMax) {
			BREncoderCalibMax = swerve.RBWheel.rotateMotor.getSensorCollection().getAnalogInRaw();
		}

		if (swerve.LFWheel.rotateMotor.getSensorCollection().getAnalogInRaw() < FLEncoderCalibMin) {
			FLEncoderCalibMin = swerve.LFWheel.rotateMotor.getSensorCollection().getAnalogInRaw();
		}
		if (swerve.LFWheel.rotateMotor.getSensorCollection().getAnalogInRaw() > FLEncoderCalibMax) {
			FLEncoderCalibMax = swerve.LFWheel.rotateMotor.getSensorCollection().getAnalogInRaw();
		}

		if (swerve.LBWheel.rotateMotor.getSensorCollection().getAnalogInRaw() < BLEncoderCalibMin) {
			BLEncoderCalibMin = swerve.LBWheel.rotateMotor.getSensorCollection().getAnalogInRaw();
		}
		if (swerve.LBWheel.rotateMotor.getSensorCollection().getAnalogInRaw() > BLEncoderCalibMax) {
			BLEncoderCalibMax = swerve.LBWheel.rotateMotor.getSensorCollection().getAnalogInRaw();
		}
		SmartDashboard.putNumber("FL Encoder Min: " , FLEncoderCalibMin);
		SmartDashboard.putNumber("FL Encoder Max: " , FLEncoderCalibMax);

		SmartDashboard.putNumber("FR Encoder Min: " , FREncoderCalibMin);
		SmartDashboard.putNumber("FR Encoder Max: " , FREncoderCalibMax);

		SmartDashboard.putNumber("BL Encoder Min: " , BLEncoderCalibMin);
		SmartDashboard.putNumber("BL Encoder Max: " , BLEncoderCalibMax);

		SmartDashboard.putNumber("BR Encoder Min: " , BREncoderCalibMin);
		SmartDashboard.putNumber("BR Encoder Max: " , BREncoderCalibMax);

		SmartDashboard.putNumber("Current FL Encoder: " , swerve.LFWheel.rotateMotor.getSensorCollection().getAnalogInRaw());
		SmartDashboard.putNumber("Current FR Encoder: " , swerve.RFWheel.rotateMotor.getSensorCollection().getAnalogInRaw());
		SmartDashboard.putNumber("Current BL Encoder: " , swerve.LBWheel.rotateMotor.getSensorCollection().getAnalogInRaw());
		SmartDashboard.putNumber("Current BR Encoder: " , swerve.RBWheel.rotateMotor.getSensorCollection().getAnalogInRaw());

	}
	
	public void activateControl(){
		swerve.setAutonomousBoolean(false);
		swerve.LBWheel.rotateMotor.setSelectedSensorPosition(swerve.LBWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.RBWheel.rotateMotor.setSelectedSensorPosition(swerve.RBWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.LFWheel.rotateMotor.setSelectedSensorPosition(swerve.LFWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.RFWheel.rotateMotor.setSelectedSensorPosition(swerve.RFWheel.rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		swerve.setPID();
		climbBar.controlDropBar();
		
		SmartDashboard.putNumber("Back Ultraboi!", swerve.ultraSonicSensors.getDistance(3));
		
		positionalIndex = 7;
		
		if (positionalOnes.get()) {
			positionalIndex -= 1;
		}
		if (positionalTwos.get()) {
			positionalIndex -= 2;
		}
		if (positionalFours.get()) {
			positionalIndex -= 4;
		}
		if (positionalIndex == 8) {
			positionalIndex = 0;
		}
		positionalIndex += 8;
		positionalIndex%=8;
		programIndex = 6;

		if (programOnes.get()) {
			programIndex -= 1;
		}
		if (programTwos.get()) {
			programIndex -= 2;
		}
		if (programFours.get()) {
			programIndex -= 4;
		}
		if (programIndex == 8) {
			programIndex = 0;
		}
		programIndex += 8;
		programIndex %= 8;
		//SmartDashboard.putNumber("Pos Index: " , positionalIndex);
		//SmartDashboard.putNumber("Pro Index: ", programIndex);
		
		//System.out.println("Dial 1: " + positionalIndex);
		//System.out.println("Dial 1: " + programIndex);
		
		//SmartDashboard.putNumber("Navx", swerve.ahrs.getYaw());
		
		//   *_*_*_*_*_*_*_* SHARED MAIN CONTROLS *_*_*_*_*_*_*_*
		
		/*if(driver.isStartHeld() && shooter.isStartHeld()){
			climbBar.setState(false);
		}else{
			climbBar.setState(true);
		}
		*/
		
		
		//   *_*_*_*_*_*_*_* DRIVER MAIN CONTROLS *_*_*_*_*_*_*_*
		if (driver.isLBHeld() || lifter.getPosition() < 10) {
			swerve.sniper();
		} else if (driver.isRBHeld() && lifter.getPosition() > 15) {
			swerve.turbo();
		} else {
			swerve.normalSpeed();
		}
		
		if (driver.getRawAxis(Rtrigger) > .1) {
			swerve.isFieldCentric = true;
			swerve.calculateSwerveControl(-driver.getRawAxis(LY), driver.getRawAxis(LX), driver.getRawAxis(RX));
		} else if (driver.getRawAxis(Ltrigger) > .1) {
			swerve.isFieldCentric = false;
			swerve.setRotateDistance(40);
			swerve.calculateObjectControl(driver.getRawAxis(RX));
		} else {
			swerve.isFieldCentric = false;
			swerve.calculateSwerveControl(-driver.getRawAxis(LY), driver.getRawAxis(LX), driver.getRawAxis(RX));
		}

		
		//   *_*_*_*_*_*_*_* SHOOTER MAIN CONTROLS *_*_*_*_*_*_*_*
		SmartDashboard.putNumber("Actuator Position", lifter.getPosition());
		if(shooter.getRawAxis(Ltrigger)>.1)
			lifter.goToPosition(27.5);
		else if(shooter.getRawAxis(Rtrigger)>.1)
			lifter.goToPosition(2.5);
		else
			lifter.goToPosition(lifter.getPosition()); 
		/*
		if(shooter.isAHeld()){
			lifter.setProportional(.4);
		}
		else{
			lifter.setProportional(.05);;
		}*/
		
		if(Math.abs(shooter.getRawAxis(LY)) > .1){
			grabber.set(shooter.getRawAxis(LY));
		}else if(shooter.isRBHeld()){
			grabber.importCube();
		}else if(shooter.isLBHeld()){
			grabber.exportCube();
		}else{
			grabber.set(0);
		}
		/*if(shooter.isBackHeld()){
			lifter.setMaxSpeed(1);
		}*/
		
		
		
	}
	
	
}
