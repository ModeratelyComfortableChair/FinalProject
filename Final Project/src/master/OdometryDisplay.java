/* DPM Team 12
 * 
 */

package master;

import lejos.hardware.lcd.TextLCD;

/** 
 * OdometryDisplay handles displaying the information from Odometry on the EV3's LCD screen.
 * 
 * @author Jerome Marfleet
 * @author Yu-Yueh Liu
 * @author Brennan Laforge
 * @version 1.0
 * @since 2016-11-06
 *
 */
public class OdometryDisplay extends Thread {
	private static final long DISPLAY_PERIOD = 250;
	private Odometer odometer;
	private TextLCD t;

	// constructor
	/**
	 * Constructor
	 * 
	 * @param odometer Robot's odometer
	 * @param t Robot's LCD
	 */
	public OdometryDisplay(Odometer odometer, TextLCD t) {
		this.odometer = odometer;
		this.t = t;
	}

	// run method (required for Thread)
	/**
	 * Method that extends from Thread class
	 * 
	 * Displays the Robot's X, Y and Theta values of Odometer on the LCD screen
	 */
	public void run() {
		long displayStart, displayEnd;
		double[] position = new double[3];

		// clear the display once
		t.clear();

		while (true) {
			displayStart = System.currentTimeMillis();

			// clear the lines for displaying odometry information
			t.drawString("X:              ", 0, 0);
			t.drawString("Y:              ", 0, 1);
			t.drawString("T:              ", 0, 2);

			// get the odometry information
			odometer.getPosition(position, new boolean[] { true, true, true });

			// display odometry information
			for (int i = 0; i < 3; i++) {
				t.drawString(formattedDoubleToString(position[i], 2), 3, i);
			}

			// throttle the OdometryDisplay
			displayEnd = System.currentTimeMillis();
			if (displayEnd - displayStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that OdometryDisplay will be interrupted
					// by another thread
				}
			}
		}
	}
	
	/**
	 * This method formats a double to a String
	 * 
	 * @param x - The double value that is being formatted into a String.
	 * @param places - The number of decimal places the String will have after the double is formatted.
	 * @return formatted String
	 */
	private static String formattedDoubleToString(double x, int places) {
		String result = "";
		String stack = "";
		long t;
		
		// put in a minus sign as needed
		if (x < 0.0)
			result += "-";
		
		// put in a leading 0
		if (-1.0 < x && x < 1.0)
			result += "0";
		else {
			t = (long)x;
			if (t < 0)
				t = -t;
			
			while (t > 0) {
				stack = Long.toString(t % 10) + stack;
				t /= 10;
			}
			
			result += stack;
		}
		
		// put the decimal, if needed
		if (places > 0) {
			result += ".";
		
			// put the appropriate number of decimals
			for (int i = 0; i < places; i++) {
				x = Math.abs(x);
				x = x - Math.floor(x);
				x *= 10.0;
				result += Long.toString((long)x);
			}
		}
		
		return result;
	}

}
