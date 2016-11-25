package master;
import lejos.hardware.Button;
import lejos.robotics.RegulatedMotor;

/**
 * Escape checks if the button Escape has been pressed in order
 * to force exit any running program
 * 
 * @author Yu-Yueh Liu
 * @version 1.0
 * @since 2016-11-07
 *
 *
 */
public class Escape {

	/**
	 * This method waits for the Escape button to be pressed
	 */
	public static void testForEscape(RegulatedMotor[] motors){
		while(Button.waitForAnyEvent() != Button.ID_ESCAPE);
		for(RegulatedMotor motor : motors){
			motor.close();
		}
		System.exit(-1);
	}
	
	public static void escape(RegulatedMotor[] motors){
		for(RegulatedMotor motor : motors){
			motor.close();
		}
		System.exit(0);
	}
}
