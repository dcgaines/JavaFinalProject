package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * AP JAVA FINAL PROJECT 2K16
 * 
 * Drives a four-wheel independent drivebase in which each module operates on
 * two axes.
 * 
 * @author Evan de Jesus & Dylan Gaines
 * @version 05/08/2016
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
	/**
	 * encoder values for correctly oriented turning wheels.
	 */
	int[] straights = { 468, 545, 531, 387 };
	/**
	 * current encoder values for turning wheels.
	 */
	int[] positions = { 0, 0, 0, 0 };

	/**
	 * Calc project objects for collecting data from turning motor.
	 */

	private final double conversion = 1023 / 270.;

	/**
	 * This code executes upon first boot of the RoboRio. Used to initialize
	 * actuators, sensors, and other objects.
	 */
	public void robotInit() {

		for (int i = 0; i < 4; i++) {
			turns[i] = new CANTalon(i);
			turns[i].setFeedbackDevice(FeedbackDevice.AnalogEncoder);
			turns[i].setProfile(1);

			drives[i] = new Victor(i);
		}

		driver = new Joystick(0);

	}

	/**
	 * This code executes once upon enabling the robot in teleop (user control).
	 */
	public void teleopInit() {
		for (int i = 0; i < turns.length; i++)
			turns[i].changeControlMode(TalonControlMode.Position);

		turnPos = turns[1].getPosition();
	}

	/**
	 * This code executes periodically every 20ms while enabled in teleop.
	 */
	public void teleopPeriodic() {

		if (driver.getMagnitude() > 0.2)
			turnPos = directionToTicks(driver.getDirectionDegrees());

		// turns[1].set((turnPos + straights[0]) % 1365);
		//
		// drives[1].set(-driver.getMagnitude());
		//
		// for(int i = 0; i < turns.length; i++){

		turns[1].set((turnPos));

		// drives[i].set(-driver.getMagnitude());
		// }

		SmartDashboard.putString("DB/String 0", "Position: " + Double.toString(turns[1].getPosition()));
		SmartDashboard.putString("DB/String 1", "turnPos: " + Double.toString(turnPos));
		SmartDashboard.putString("DB/String 5",
				"Pos in degrees: " + Double.toString(ticksToDegrees(turns[1].getPosition())));
		SmartDashboard.putString("DB/String 4", "Set value: " + Double.toString((turnPos + straights[1]) % 1365));

	}

	/**
	 * This code is used to output the encoder values of the turning motors.
	 */
	public void testPeriodic() {
		turns[1].changeControlMode(TalonControlMode.PercentVbus);
		SmartDashboard.putString("DB/String 3", "Stick: " + Double.toString(driver.getDirectionDegrees()));
		SmartDashboard.putString("DB/String 4",
				"Stick: " + Double.toString(directionToTicks(driver.getDirectionDegrees())));

		SmartDashboard.putString("DB/String 0", "FL: " + Double.toString(turns[1].getPosition()));
		SmartDashboard.putString("DB/String 1", "FR: " + Double.toString(turns[1].getPosition()));
		SmartDashboard.putString("DB/String 2", "BL: " + Double.toString(turns[1].getPosition()));
		// SmartDashboard.putString("DB/String 3", "BR: " +
		// Double.toString(turns[1].getPosition()));
	}

	/**
	 * Removes idle joystick input, as PS4 controllers have an idle input of
	 * about 8%. Removing this input gets rid of Victor whining and validates
	 * talon standby.
	 * 
	 * @param input
	 *            value to be checked for zeroing.
	 * @return throttle based on threshold criteria.
	 */
	public double deadband(double input) {

		double threshold = 0.08;

		if (input < 0 && input > -threshold)
			return 0;
		else if (input > 0 && input < threshold)
			return 0;

		return input;
	}

	/**
	 * Converts joystick angle to encoder ticks via a proportionality constant.
	 * 
	 * @param input
	 *            degrees to be converted to encoder ticks.
	 * @return value encoder tick value.
	 */
	public int directionToTicks(double input) {
		if (input < 0)
			input += (360);

		// SmartDashboard.putString("DB/String 2", "Stick: " +
		// Double.toString(input));

		input *= conversion;

		return 1365 - (int) input;
	}

	/**
	 * Gives talon encoder value in degrees.
	 * 
	 * @param input
	 *            ticks to be converted to degrees.
	 * @return talon motor position in degrees.
	 */
	public double ticksToDegrees(double input) {
		return input * (1 / conversion);
	}
}
