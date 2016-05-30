package org.usfirst.frc.team217.robot;

import java.io.*;

public class Calcs {

	public static void main(String[] args) throws Exception {
		PrintWriter writer = new PrintWriter(new File("calcs.txt"));
		double a_cl;
		double length = 31.0, width = 20.625;
		for (int i = 1; i <= 10; i++) {
			a_cl = i * 6.75;
			double r_cl = length / (2.0 * Math.sin(a_cl * (Math.PI / 180.0)));
			double r_cp = (length / 2.0) / Math.tan(a_cl * (Math.PI / 180.0));
			double a_o, a_i = 0;
			a_o = (180 / Math.PI) * Math.atan((length / 2.0) / (r_cp + (width / 2.0)));
			if (r_cp > (width / 2)) {
				a_i = (180 / Math.PI) * Math.atan((length / 2.0) / (r_cp - (width / 2.0)));
			} else if (r_cp < (width / 2)) {
				a_i = Math.PI - ((180 / Math.PI) * Math.atan((length / 2.0) / (r_cp - (width / 2.0))));
			}

			double r_i = length / (2.0 * Math.sin(a_i * (Math.PI / 180.0)));
			double r_o = length / (2.0 * Math.sin(a_o * (Math.PI / 180.0)));
			writer.printf("%.3f\n", r_i / r_o);
		}

		writer.close();
	}
}