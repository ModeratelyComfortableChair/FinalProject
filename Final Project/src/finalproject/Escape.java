package finalproject;
import lejos.hardware.Button;

public class Escape {

	public static void testForEscape(){
		while(Button.waitForAnyEvent() != Button.ID_ESCAPE);
		System.exit(-1);
	}
}
