/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3373.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.IterativeRobot;
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
	
	double position;
	
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	int counter = 0;
	
	
	//SupremeTalon talon1;
	//SupremeTalon talon2;
	
	//Dual Linear Actuator Configs
	//Look at Actuator.calibrate to view documentaion about how to calculate individual Actuators
	static int actuator1Port1 = 7;
	static int actuator1Port2 = 8;
	static int actuator2Port1 = 10;
	static int actuator2Port2 = 9;
	static double minPot1 = 97;
	static double minPot2 = 95;
	static double maxPot1 = 725;
	static double maxPot2 = 726;
	static double minDistance1 = 2;
	static double minDistance2 = 2;
	static double maxDistance1 = 26.5;
	static double maxDistance2 = 26.5;
	
	//Grabber Initialization
	int grabberPort1 =0; // Need to updtate for the Robot
	int grabberPort2 =0;
	Grabber grabber;
	
	
	
	//Values for SuperJoystick getRawAxis()
	int LX = 0;
	int LY = 1;
	int Ltrigger = 2;
	int Rtrigger = 3;
	int RX = 4;
	int RY = 5;
	
	Actuator act1;
	
	DualActuators actuators;
	
	SupremeTalon sim;
	
	SuperJoystick shooter;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		
		
		act1 = new Actuator(actuator1Port1, actuator1Port2, maxPot1, minPot1, maxDistance1, minDistance1);
		
		position = 6;
		counter = 0;
		this.setPeriod(.01);
		actuators = new DualActuators(actuator1Port1,actuator2Port1,actuator1Port2,actuator2Port2,maxPot1,maxPot2,minPot1,minPot2,maxDistance1,maxDistance2,minDistance1,minDistance2);
		shooter = new SuperJoystick(0);
		grabber = new Grabber(grabberPort1,grabberPort2);
		
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
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
		String locations = DriverStation.getInstance().getGameSpecificMessage();
		if(locations.charAt(0) == 'L'){
			System.out.println("LeftSide Switch");
		}else{
			System.out.println("RightSide Switch");
		}
		if(locations.charAt(1) == 'L'){
			System.out.println("LeftSide Scale");
		}else{
			System.out.println("RightSide Scale");
		}
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// go r0idefaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		
		/*actuator1.set(shooter.getRawAxis(LY));
		actuator2.set(shooter.getRawAxis(RY));
		SmartDashboard.putNumber("Pot1", actuator1.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Pot2", actuator2.getSelectedSensorPosition(0));
		/*
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;*/
		//}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		//Lift Code for Actuators Moving Together
		if(shooter.getRawAxis(Rtrigger)>.1)
			actuators.goToPosition(26.5);
		else if(shooter.getRawAxis(Ltrigger)>.1)
			actuators.goToPosition(3);
		else
			actuators.goToPosition(actuators.getPosition());
		
		//Grabber Code
		if(shooter.isLBHeld())
			grabber.exportCube();
		else if(shooter.isRBHeld())
			grabber.importCube();
		else if(grabber.hasCube())
			grabber.keepCube();
		else
			grabber.idle();
			
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		actuators.calibrate(shooter, false);
		actuators.calibrate(shooter, true);
	}
}
