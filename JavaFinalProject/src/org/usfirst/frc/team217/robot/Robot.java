package org.usfirst.frc.team217.robot;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * AP JAVA FINAL PROJECT 2K16
 * 
 * @author Evan de Jesus & Dylan Gaines
 * @version 05/08/2016
 */

public class Robot extends IterativeRobot {

	CANTalon[] turns = new CANTalon[4];
	Victor[] drives = new Victor[4];
	Joystick driver;
	double lastValue, turnPos;
	int[] straights = { 468, 545, 531, 387 };
	int[] positions = { 0, 0, 0, 0 };

	PrintWriter inputWriter, outputWriter;
	File f1, f2;
	Timer autonTimer;
	double calcSpeed = 0;

	final double conversion = 1023 / 270.;

	public void robotInit() {

		f1 = new File("/media/sda1/calc/input.txt");
		f2 = new File("/media/sda1/calc/speed.txt");
		try {
			inputWriter = new PrintWriter(f1);
			outputWriter = new PrintWriter(f2);
		} catch (IOException e) {
			System.out.println("File(s) not found!");
		}

		for (int i = 0; i < 4; i++) {
			turns[i] = new CANTalon(i);
			turns[i].setFeedbackDevice(FeedbackDevice.AnalogEncoder);
			turns[i].setProfile(1);

			drives[i] = new Victor(i);
		}

		driver = new Joystick(0);

		autonTimer = new Timer();
	}

	public void autonomousInit() {
		autonTimer.reset();
		autonTimer.start();
		for (int i = 0; i < 4; i++) {
			turns[i].changeControlMode(TalonControlMode.PercentVbus);
		}
		inputWriter.println("Time" + "\t" + "Input");
		outputWriter.println("Time" + "\t" + "Speed");
	}

	public void autonomousPeriodic() {
		double time = autonTimer.get();

		if (time <= 24) {
			if (time < 6.568) {
				// turns[0].set(Math.sin(time));
				calcSpeed = Math.sin(time);
			} else if (time >= 6.568 && time < 16) {
				// turns[0].set(0.5*(time * time) - 1);
				calcSpeed = 0.5 * Math.pow(time, 0.5) - 1;
			} else if (time >= 6.568 && time <= 24) {
				// turns[0].set(-0.5 * Math.pow((time - 16),2) + 1);
				calcSpeed = -0.05 * Math.pow(time - 16, 2) + 1;
			}

			if (calcSpeed > 1)
				calcSpeed = 1;
			if (calcSpeed < -1)
				calcSpeed = -1;

			turns[0].set(calcSpeed);

			inputWriter.printf("%.3f	%.3f\n", time, calcSpeed);
			outputWriter.printf("%.3f	%.3f\n", time, turns[0].getSpeed());
		} else {
			turns[0].set(0);
			inputWriter.close();
			outputWriter.close();
		}
	}

	public void teleopInit() {
		for (int i = 0; i < turns.length; i++)
			turns[i].changeControlMode(TalonControlMode.Position);

		lastValue = 0;
		turnPos = turns[1].getPosition();
	}

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

	public double deadband(double input) {

		double threshold = 0.08;

		if (input < 0 && input > -threshold)
			return 0;
		else if (input > 0 && input < threshold)
			return 0;

		return input;
	}

	public double absVal(double num) {
		if (num < 0)
			return -num;
		else
			return num;
	}

	public int directionToTicks(double input) {
		if (input < 0)
			input += (360);

		// SmartDashboard.putString("DB/String 2", "Stick: " +
		// Double.toString(input));

		input *= conversion;

		return 1365 - (int) input;
	}

	public double ticksToDegrees(double input) {
		return input * (1 / conversion);
	}

}
