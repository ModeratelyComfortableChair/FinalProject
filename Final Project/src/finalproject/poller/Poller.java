package finalproject.poller;
import lejos.robotics.SampleProvider;
public abstract class Poller extends Thread{

	protected SampleProvider sensor;
	protected float[] data;
	public Poller(SampleProvider sensor, float[] data){
		this.sensor = sensor;
		this.data = data;
	}
	public abstract double filterData();
}
