package finalproject.poller;

import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

public class USPoller extends Poller {

	private UltrasonicController cont;
	private int distance;
	private static final int REFRESH_TIME_MS = 50;

	public USPoller(SampleProvider sensor, float[] data) {
		super(sensor, data);
	}

	public void run(){
		int distance;
		while (true) {
			sensor.fetchSample(data,0);							// acquire data
			distance=(int)(data[0]*100.0);					// extract from buffer, cast to int
			try { Thread.sleep(50); } catch(Exception e){}		// Poor man's timed sampling
		}
	}

	public double readUSDistance() {
		return this.distance;
	}

	@Override
	public double filterData(){
		return 0;
	}

}
