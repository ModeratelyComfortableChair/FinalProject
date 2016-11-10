package finalproject;
import lejos.hardware.Button;

public class Escape extends Thread{

	@Override
	public void run(){
		while(Button.waitForAnyEvent() != Button.ID_ESCAPE);
		System.exit(-1);
	}
}
