package master.poller;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightPoller extends Poller {

	private EV3ColorSensor sensor;
	public LightPoller(SensorModes sensor) {
		super(sensor);
		// TODO Auto-generated constructor stub
	}


	@Override
	public double filterData() {
		// TODO Auto-generated method stub
		return 0;
	}

}
