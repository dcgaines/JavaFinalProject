package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * AP JAVA FINAL PROJECT 2K16
 * 
 * @author Evan de Jesus & Dylan Gaines
 * @version 05/08/2016
 */


public class Robot extends IterativeRobot {
	
	CANTalon FLTurn, FRTurn, BLTurn, BRTurn;
	Victor FLDrive, FRDrive, BLDrive, BRDrive;
	Joystick driver;
	AnalogInput FLEncoder, FREncoder, BLEncoder, BREncoder;
	double max, min;
	
    public void robotInit() {
    	FLTurn = new CANTalon(3);
    	FRTurn = new CANTalon(1);
    	BLTurn = new CANTalon(2);
    	BRTurn = new CANTalon(0);
    	
    	FLDrive = new Victor(3);
    	FRDrive = new Victor(1);
    	BLDrive = new Victor(2);
    	BRDrive = new Victor(0);
    	
    	driver = new Joystick(0);
    	
    	FLEncoder = new AnalogInput(3);
    	FREncoder = new AnalogInput(1);
    	BLEncoder = new AnalogInput(2);
    	BREncoder = new AnalogInput(0);
    	
    }
    
    public void autonomousInit() {
    	
    }

    public void autonomousPeriodic() {
    	
    	
    }
    
    public void teleopInit(){
    	max = 0;
    	min = FLEncoder.getValue();
    	
    	FLTurn.configMaxOutputVoltage(1);
    }

    public void teleopPeriodic() {
        turnAll(deadband(driver.getZ()));
        driveAll(-deadband(driver.getY()));
    }
    
    public void testPeriodic() {
    	
    }
    
    public void driveAll(double input){
        FLDrive.set(input);
        FRDrive.set(input);
        BLDrive.set(input);
        BRDrive.set(input);
    }
    
    public void turnAll(double input){
    	FLTurn.set(input);
    	FRTurn.set(input);
    	BLTurn.set(input);
    	BRTurn.set(input);
    }
    
    public double deadband(double input){
    	
    	double threshold = 0.08;
    	
    	if(input < 0 && input > -threshold) return 0;
    	else if(input > 0 && input < threshold) return 0;
    	
    	return input;
    }
    
}
