package poller;

import lejos.robotics.SampleProvider;

public class USPoller extends Poller {

	
	public USPoller(SampleProvider sensor, float[] data) {
		super(sensor, data);
	}

	@Override
	public float getSensorInformation() {
		// TODO Auto-generated method stub
		return 0;
	}

}
