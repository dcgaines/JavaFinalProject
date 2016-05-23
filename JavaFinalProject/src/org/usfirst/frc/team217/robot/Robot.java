package org.usfirst.frc.team217.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

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

    int session;
    Image frame;
    
    public void robotInit() {
    	
        frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

        // the camera name (ex "cam0") can be found through the roborio web interface
        session = NIVision.IMAQdxOpenCamera("cam2",
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(session);
    	
    	FLTurn = new CANTalon(3);
    	FRTurn = new CANTalon(1);
    	BLTurn = new CANTalon(2);
    	BRTurn = new CANTalon(0);
    	
    	FLDrive = new Victor(3);
    	FRDrive = new Victor(1);
    	BLDrive = new Victor(2);
    	BRDrive = new Victor(0);
    	
    	driver = new Joystick(0);

    	
    }
    
    public void autonomousInit() {
    }

    public void autonomousPeriodic() {

    }
    
    public void teleopInit(){
    	
    	NIVision.IMAQdxStartAcquisition(session);
    }

    public void teleopPeriodic() {
    	
    	 NIVision.IMAQdxStartAcquisition(session);
    	 
         NIVision.IMAQdxGrab(session, frame, 1);
         CameraServer.getInstance().setImage(frame);

         tankDrive();

        Timer.delay(0.005);
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
    
    public void tankDrive(){
       	
        FLDrive.set(deadband(-driver.getY()));
        FRDrive.set(deadband(-driver.getRawAxis(5)));
        BLDrive.set(deadband(driver.getY()));
        BRDrive.set(-deadband(-driver.getRawAxis(5)));
    }
    
    public double deadband(double input){
    	
    	double threshold = 0.08;
    	
    	if(input < 0 && input > -threshold) return 0;
    	else if(input > 0 && input < threshold) return 0;
    	
    	return input;
    }
    
    public double absVal(double num){
    	if(num < 0)
    		return -num;
    	else
    		return num;
    }
    
}
