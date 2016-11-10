package master.poller;

import lejos.robotics.SampleProvider;

public class LightPoller extends Poller {

	public LightPoller(SampleProvider sensor, float[] data) {
		super(sensor, data);
		// TODO Auto-generated constructor stub
	}


	@Override
	public double filterData() {
		// TODO Auto-generated method stub
		return 0;
	}

}
