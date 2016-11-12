package master.poller;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
public abstract class Poller extends Thread{

	protected SensorModes sensor;
	protected float[] data;
	protected SampleProvider value;
	public abstract double filterData();
	public Poller(SensorModes sensor){
		this.sensor = sensor;
	}
}