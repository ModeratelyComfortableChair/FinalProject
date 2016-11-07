package finalproject.poller;

import lejos.robotics.SampleProvider;

public class USPoller extends Poller {
	
	private UltrasonicController cont;
	private int distance;
	
	public USPoller(SampleProvider sensor, float[] data, UltrasonicController cont) {
		//super(sensor, data); //what is this for?
		this.cont=cont;
	}

	@Override
	public float getSensorInformation() {
		// TODO Auto-generated method stub
		return 0;
	}
	
	public double readUSDistance() {
		return this.distance;
	}
	public void processUSData(double distance) {
		//TODO:
	}

}
