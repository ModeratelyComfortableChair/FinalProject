package master.poller;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class USPoller extends Poller {

	private int distance;
	private static final int REFRESH_TIME_MS = 50;
	private boolean enabled, updated;
	private EV3UltrasonicSensor sensor;

	public USPoller(SensorModes sensor) {
		super(sensor);
		this.sensor = (EV3UltrasonicSensor) sensor;
		disable();
		updated = false;
	}

	public void run(){
		while (true) {
			if(enabled){
				sensor.fetchSample(data,0);							// acquire data
				distance=(int)(data[0]*100.0);					// extract from buffer, cast to int
				try { Thread.sleep(50); } catch(Exception e){}		// Poor man's timed sampling
				updated = !updated;
			} else {
				distance = 0;
			}
		}
	}

	/**
	 * Method returns distance obtained by filter. Will not return until poller updates distance.
	 * @return double value of USSensor
	 */
	@Override
	public double filterData() {
		boolean current = updated;
		System.out.print("");
		while(current == updated);
		return distance;
	}

	
	/**
	 * Sets sensor to getDistanceMode and initializes the value and data. Sets enabled to true
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

	/**
	 * Sets sensor to getListenMode so that it does not send pings. Sets enabled to false;
	 */
	public void disable(){
		sensor.getListenMode();
		enabled = false;
	}


}
