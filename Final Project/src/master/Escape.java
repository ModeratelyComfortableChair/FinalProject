package master;
import lejos.hardware.Button;

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
	public static void testForEscape(){
		while(Button.waitForAnyEvent() != Button.ID_ESCAPE);
		System.exit(-1);
	}
}
