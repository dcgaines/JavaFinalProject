package org.usfirst.frc.team217.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	/**
	 * Array of turning motor talons.
	 */
	CANTalon[] turns = new CANTalon[4];
	/**
	 * Array of driving motor victors. Same addresses as talons.
	 */
	Victor[] drives = new Victor[4];
	/**
	 * PS4 controller used to operate robot.
	 */
	Joystick driver;
	double turnPos;
	double joyStickAngle, turnAngle, encTicks, encDeg;
	boolean isReversed;
	double driveSpeed = 0;
	
    int session;
    Image frame;


	/**
	 * encoder values for correctly oriented turning wheels.
	 */
	int[] straights = { 461, 539, 525, 387 };

	/**
	 * current encoder values for turning wheels.
	 */
	int[] positions = { 0, 0, 0, 0 };

	double [] turnTargets = {236.203,123.797,303.797,56.203};
	
	private final double conversion = 1023 / 270.;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		for (int i = 0; i < 4; i++) {
			turns[i] = new CANTalon(i);
			turns[i].setFeedbackDevice(FeedbackDevice.AnalogPot);
			turns[i].setProfile(1);

			drives[i] = new Victor(i);
			turns[i].setP(2.1);
		}

		driver = new Joystick(0);
		
        frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

        // the camera name (ex "cam0") can be found through the roborio web interface
        session = NIVision.IMAQdxOpenCamera("cam1",
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(session);

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

	}

	public void teleopInit() {
		for (int i = 0; i < turns.length; i++)
			turns[i].changeControlMode(TalonControlMode.Position);

		turnPos = turns[1].getPosition();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
        NIVision.IMAQdxGrab(session, frame, 1);
        CameraServer.getInstance().setImage(frame);
		
		if (deadband(driver.getZ()) == 0) {
			if (driver.getMagnitude() > .25) {
				for (int i = 0; i < 4; i++) {
					turns[i].set((__(driver.getDirectionDegrees()) + straights[i]) % (1365));
				}

			}
			printToDash();

			driveSpeed = -deadband(driver.getMagnitude());
			if (isReversed)
				driveSpeed *= -1;
			for (int i = 0; i < 4; i++) {
				if (i == 2)
					drives[i].set(-driveSpeed);
				else
					drives[i].set(driveSpeed);
			}
		} else if(deadband(driver.getZ()) != 0){
			for(int i = 0; i < 4; i++){
				turns[i].set((__(turnTargets[i]) + straights[i]) % 1365);
				driveSpeed = -deadband(driver.getZ());
				if (isReversed)
					driveSpeed *= -1;
					if (i == 2)
						drives[i].set(-driveSpeed);
					else
						drives[i].set(driveSpeed);
				
			}
		}
		else{
			for(int i = 0; i < 4; i++){
				drives[i].set(0);
			}
		}
        Timer.delay(0.005);		// wait for a motor update time
		
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		printToDash();
	}

	public void printToDash() {
		SmartDashboard.putString("DB/String 0",
				Double.toString(ticksToDegrees(turns[1].getPosition())));

		SmartDashboard.putString("DB/String 5",
				"0: " + Double.toString(turns[0].getPosition()));
		SmartDashboard.putString("DB/String 6",
				"1: " + Double.toString(turns[1].getPosition()));
		SmartDashboard.putString("DB/String 7",
				"2: " + Double.toString(turns[2].getPosition()));
		SmartDashboard.putString("DB/String 8",
				"3: " + Double.toString(turns[3].getPosition()));
	}

	public double ticksToDegrees(double tick) {
		return tick / conversion;
	}

	public int degreeToTick(double degree) {
		return (int) (degree * conversion);
	}

	public double deadband(double input) {
		if (input < .08 && input > -.08)
			return 0;
		else
			return input;
	}
	
	public double __(double input){
		joyStickAngle = input;
		if (joyStickAngle < 0)
			joyStickAngle += 360;
		joyStickAngle = 360 - joyStickAngle;

		if (joyStickAngle < 270 && joyStickAngle > 90) {
			joyStickAngle += 180;
			isReversed = true;
		} else {
			isReversed = false;
		}
		if (joyStickAngle > 360)
			joyStickAngle -= 360;
		turnPos = degreeToTick(joyStickAngle);

		return turnPos;
	}
}
