package org.usfirst.frc.team217.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

/**
 * JAVA FINAL PROJECT 2K16
 * 
 * This program operates a four-wheel independent drivebase in which each wheel
 * operates on two axes. Wheel angle is monitored by absolute potentiometers
 * feeding into TalonSRX motor controllers. Wheels are turned with bag motors
 * and driven by mini-cim motors.
 * 
 * The modes of operation are crab drive and snake drive; when only the left
 * joystick is given throttle, the robot moves in the direction of the joystick
 * and maintains its orientation. In snake mode, the robot moves radially around
 * a point outside of the chassis with a maximum turn radius of 67.5 degrees.
 * The robot can also make a zero point turn when the right trigger is pressed.
 * 
 * @author Evan de Jesus
 * @author Dylan Gaines
 * @version 5/29/2016
 */
public class Swerve extends IterativeRobot {
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

	/**
	 * Values that are used to aid the calculations between degrees and encoder
	 * ticks.
	 */
	double joyStickAngle, turnAngle, encTicks, encDeg;

	/**
	 * Becomes true when the module flips direction in order to avoid hitting
	 * the 90 degree deadzone.
	 */
	boolean isReversed;

	/**
	 * Arrays that hold the angles and speed ratios necessary for the robot to
	 * drive in snake mode.
	 */
	double[] a_i = { 0, 7.321, 15.945, 26.055, 37.626, 50.264, 63.182, 75.476, 86.497, 87.131, 79.046 };
	double[] a_o = { 0, 6.261, 11.696, 16.5, 20.833, 24.823, 28.573, 32.172, 35.696, 39.218, 42.81 };
	double[] zs = new double[a_i.length];
	double[] ratios = { 1.000, 0.856, 0.738, 0.647, 0.583, 0.546, 0.536, 0.550, 0.585, 0.633, 0.692 };

	/**
	 * Camera variables.
	 */
	int session;
	Image frame;

	/**
	 * Encoder values for correctly oriented turning wheels.
	 */
	int[] straights = { 461, 539, 525, 387 };

	/**
	 * Angles for zero point turning.
	 */
	double[] turnTargets = { 236.203, 123.797, 303.797, 56.203 };

	private final double conversion = 1023 / 270.;

	/**
	 * This method runs when the robot is first started up and should be used
	 * for any initialization code.
	 */
	public void robotInit() {

		// Talons are calibrated for position control.
		for (int i = 0; i < 4; i++) {
			turns[i] = new CANTalon(i);
			turns[i].setFeedbackDevice(FeedbackDevice.AnalogPot);
			turns[i].setProfile(1);

			drives[i] = new Victor(i);
			turns[i].setP(2.5);
		}

		// Will be used to map the Z axis to discrete values.
		zs[0] = 0;
		for (int i = 1; i < 11; i++) {
			zs[11 - i] = 1 / i;
		}

		driver = new Joystick(0);

		// Vision code
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		session = NIVision.IMAQdxOpenCamera("cam1", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);

	}

	/**
	 * This method is called once upon the start of operator control.
	 */
	public void teleopInit() {
		for (int i = 0; i < turns.length; i++)
			turns[i].changeControlMode(TalonControlMode.Position);
	}

	/**
	 * This method is called periodically during operator control.
	 */
	public void teleopPeriodic() {
		NIVision.IMAQdxGrab(session, frame, 1);
		CameraServer.getInstance().setImage(frame);

		// Zero point turn. All wheels become tangent to the center to create an
		// ellipse.
		if (driver.getRawButton(8)) {
			for (int i = 0; i < 4; i++) {
				turns[i].set((__(turnTargets[i]) + straights[i]) % 1365);
				double turnSpeed = -deadband(driver.getZ());
				if (isReversed)
					turnSpeed *= -1;
				if (i == 2)
					drives[i].set(-turnSpeed);
				else
					drives[i].set(turnSpeed);

			}
		} /*
			 * Crab mode. All wheels follow the same direction and the chassis
			 * maintains orientation.
			 */
		else if (deadband(driver.getZ()) == 0 && deadband(driver.getMagnitude()) != 0) {
			if (driver.getMagnitude() > .1) {
				for (int i = 0; i < 4; i++) {
					/*
					 * Angle is converted so that the wheel is displaced in
					 * relation to its straight value.
					 */
					turns[i].set((__(driver.getDirectionDegrees()) + straights[i]) % (1365));
				}
			}

			double driveSpeed = -deadband(driver.getMagnitude());
			if (isReversed)
				driveSpeed *= -1;
			for (int i = 0; i < 4; i++) {
				if (i == 2)
					drives[i].set(-driveSpeed);
				else
					drives[i].set(driveSpeed);
			}
		} /*
			 * Snake mode. All wheels become tangent to a circle whose center is
			 * the rotation point. Inside wheels are slowed down based on
			 * distance ratio, and the angles of the insides wheels are sharper.
			 * 
			 * Turn radius is limited to 67.5 degrees in order to avoid wheel
			 * twitch.
			 */
		else if (deadband(driver.getZ()) != 0 && deadband(driver.getMagnitude()) != 0) {
			
			//Maps Z axis to discrete values, which have equivalent angles and speed ratios.
			int input = (int) Math.round(-driver.getZ() * 10);
			if (input < 0) {
				input = -input;
			}
			double speed_o = -driver.getY();
			double speed_i = speed_o * ratios[input];

			if (deadband(driver.getZ()) < 0) {
				turns[0].set((__(a_o[input]) + straights[0]) % 1365);
				turns[1].set((__(360 - a_o[input]) + straights[1]) % 1365);
				turns[2].set((__(a_i[input]) + straights[2]) % 1365);
				turns[3].set((__(360 - a_i[input]) + straights[3]) % 1365);

				drives[0].set(-speed_o);
				drives[1].set(-speed_o);
				drives[2].set(speed_i);
				drives[3].set(-speed_i);
			} else if (deadband(driver.getZ()) > 0) {
				turns[0].set((__(-1 * a_i[input]) + straights[0]) % 1365);
				turns[1].set((__(360 - -1 * a_i[input]) + straights[1]) % 1365);
				turns[2].set((__(-1 * a_o[input]) + straights[2]) % 1365);
				turns[3].set((__(360 - -1 * a_o[input]) + straights[3]) % 1365);

				drives[0].set(-speed_i);
				drives[1].set(-speed_i);
				drives[2].set(speed_o);
				drives[3].set(-speed_o);
			}
		} // Idle input stops all motors.
		else {
			for (int i = 0; i < 4; i++) {
				drives[i].set(0);
			}
		}
		Timer.delay(0.005); // wait for a motor update time

	}

	/**
	 * converts between encoder ticks and degrees for easier calculation.
	 * Encoder values range from 0-1023 whereas it is more intuitive to operate
	 * on a 360 degree circle.
	 * 
	 * @param tick
	 *            encoder position to be converted
	 * @return degree value of turn motor
	 */
	public double ticksToDegrees(double tick) {
		return tick / conversion;
	}

	/**
	 * Converts calculated degree to encoder position.
	 * 
	 * @param degree
	 *            value to be converted to ticks
	 * @return tick value that the motor will be set to.
	 */
	public int degreesToTicks(double degree) {
		return (int) (degree * conversion);
	}

	/**
	 * removes idle joystick input at 8% as PS4 controllers have a slight input
	 * when the joystick is in its neutral position.
	 * 
	 * @param input
	 *            joystick input to be checked
	 * @return truncated joystick input when the stick is idle
	 */
	public double deadband(double input) {

		final double threshold = 0.08;

		if (Math.abs(input) < threshold)
			return 0;

		return input;
	}

	/**
	 * Maps the joystick angle to a 360 degree circle, and restricts the motor
	 * to operate within 180 degrees, as pots have a 90 degree deadzone in which
	 * values are not recorded. This logic reverses the motor and sets the
	 * direction to the opposite of the desired angle.
	 * 
	 * @param input
	 *            angle to be converted, which will set the motor to the desired
	 *            position.
	 * @return encoder value that the turn motor will be set to.
	 */
	public double __(double input) {
		// Joystick is formatted for intuitive calculations
		joyStickAngle = input;
		if (joyStickAngle < 0)
			joyStickAngle += 360;
		joyStickAngle = 360 - joyStickAngle;

		// wheel is limited to 180 degree freedom, drive motor is reversed when
		// desired angle is outside this range
		if (joyStickAngle < 270 && joyStickAngle > 90) {
			joyStickAngle += 180;
			isReversed = true;
		} else {
			isReversed = false;
		}
		if (joyStickAngle > 360)
			joyStickAngle -= 360;
		double turnPos = degreesToTicks(joyStickAngle);

		return turnPos;
	}

}
