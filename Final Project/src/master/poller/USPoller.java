package master.poller;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class USPoller extends Poller {

	private int distance;
	private static final int REFRESH_TIME_MS = 50;
	private boolean enabled;
	private EV3UltrasonicSensor sensor;

	public USPoller(SensorModes sensor) {
		super(sensor);
		this.sensor = (EV3UltrasonicSensor) sensor;
		disable();
	}

	public void run(){
		while (true) {
			if(enabled){
				sensor.fetchSample(data,0);							// acquire data
				distance=(int)(data[0]*100.0);					// extract from buffer, cast to int
				try { Thread.sleep(50); } catch(Exception e){}		// Poor man's timed sampling
			} else {
				distance = 0;
			}
		}
	}

	/**
	 * Method returns distance obtained by
	 * @return double value of USSensor
	 */
	@Override
	public double filterData() {
		return distance;
	}

	/**
	 * Setter method for fetch. Fetch controls whether or not the sensor should be fetching values.
	 * @param fetch
	 */
	public void enable(){
		while(!sensor.isEnabled()){
			Sound.beep();
			try { Thread.sleep(1000); } catch(Exception e){}
		}
		value = sensor.getDistanceMode();
		data = new float[value.sampleSize()];
		enabled = true;
	}

	public void disable(){
		sensor.getListenMode();
		enabled = false;
	}


}
