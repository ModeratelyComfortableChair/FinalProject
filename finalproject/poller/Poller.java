package finalproject.poller;
import lejos.robotics.SampleProvider;
public abstract class Poller extends Thread{

	private SampleProvider sensor;
	private float[] data;
	public abstract float getSensorInformation();
	public Poller(SampleProvider sensor, float[] data){
		this.sensor = sensor;
		this.data = data;
	}
}
