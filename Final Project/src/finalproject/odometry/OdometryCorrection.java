package finalproject.odometry;

import master.Odometer;
import master.poller.LightPoller;

public class OdometryCorrection {

	private LightPoller leftLight, rightLight;
	public OdometryCorrection(Odometer odometer, LightPoller leftLight, LightPoller rightLight){
		this.leftLight = leftLight;
		this.rightLight = rightLight;
	}
}
