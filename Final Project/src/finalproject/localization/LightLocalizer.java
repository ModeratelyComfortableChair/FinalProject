package finalproject.localization;

import finalproject.Navigation;
import finalproject.Odometer;
import finalproject.poller.LightPoller;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightLocalizer implements Localizer {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	public static final int ROTATION_SPEED = 60;
	private static final double COLOUR_BARRIER = 0.28;
	public static final double X_CORR = 6.5;
	public static final double Y_CORR = 6.5;
	public static final double TILE_WIDTH = 30.48;
	private static final double SENSOR_DISTANCE = 14;
	private Navigation nav;
	private LightPoller lightPoller;
	private double color;
	
	public LightLocalizer(Odometer odo, LightPoller lightPoller, Navigation nav) {
		this.odo = odo;
		this.lightPoller = lightPoller;
		this.nav = nav;
	}
	@Override
	public void localize() {
		double thetaX1, thetaX2, thetaY1, thetaY2, tempAngle, x, y;
		thetaX1 = thetaX2 = thetaY1 = thetaY2 = -12312;
		boolean xFirst, yAbove;
		yAbove = false;
		nav.travelTo(0,0); 													//Travel to (0,0) and fix orientation to 0 degrees
		nav.turnTo(-nav.getOrientation());
		nav.setOrientation((int)(odo.getTheta()*(180.0/Math.PI)));
		
		
		nav.rotateOnSpot(ROTATION_SPEED);
		color = getFilteredData();								//Rotate until grid line
		while(color > COLOUR_BARRIER){
			color = getFilteredData();
		}
		Sound.beep();
		nav.stopMotors();
		tempAngle = (odo.getTheta()*(180.0/Math.PI));
		if(tempAngle > 45){													//Determine where robot is with respect to center
			thetaX1 = tempAngle;
			xFirst = true;
			if(tempAngle > 90){
				yAbove = false;
			} else {
				yAbove = true;
			}
		} else {
			thetaY1 = tempAngle;
			xFirst = false;
		}
		nav.rotateOnSpot(ROTATION_SPEED);
		try {Thread.sleep(1000);} catch (InterruptedException e) {}
		color = getFilteredData();								//Rotate until grid line
		while(color > COLOUR_BARRIER){
			color = getFilteredData();
		}
		Sound.beep();
		nav.stopMotors();
		
		if(xFirst){
			thetaY1 = (odo.getTheta()*(180.0/Math.PI));
		} else {
			thetaX1 = (odo.getTheta()*(180.0/Math.PI));
			if((odo.getTheta()*(180.0/Math.PI)) > 90){
				yAbove = true;
			} else {
				yAbove = false;
			}
		}
		
		nav.rotateOnSpot(ROTATION_SPEED);
		try {Thread.sleep(1000);} catch (InterruptedException e) {}
		color = getFilteredData();	//Fetch colour
		while(color > COLOUR_BARRIER){
			color = getFilteredData();
		}
		Sound.beep();
		nav.stopMotors();
		
		if(xFirst){
			thetaX2 = (odo.getTheta()*(180.0/Math.PI));
		} else {
			thetaY2 = (odo.getTheta()*(180.0/Math.PI));
		} 
		nav.rotateOnSpot(ROTATION_SPEED);
		try {Thread.sleep(1000);} catch (InterruptedException e) {}
		color = getFilteredData();	//Fetch colour
		while(color > COLOUR_BARRIER){
			color = getFilteredData();
		}
		Sound.beep();
		nav.stopMotors();
		
		if(xFirst){
			thetaY2 = (odo.getTheta()*(180.0/Math.PI));
		} else {
			thetaX2 = (odo.getTheta()*(180.0/Math.PI));
		}
		nav.setOrientation((int)(odo.getTheta()*(180.0/Math.PI)));
		
		if(xFirst){
			if(yAbove){
				x = SENSOR_DISTANCE*Math.cos(((Math.PI / 180.0)*(thetaY2 - thetaY1)/2));			//Calculate x and y 
				y = SENSOR_DISTANCE*Math.cos(((Math.PI / 180.0)*(thetaX1)));
			} else {
				x = SENSOR_DISTANCE*Math.cos(((Math.PI / 180.0)*(thetaY2 - thetaY1)/2));
				y = -SENSOR_DISTANCE*Math.cos(((Math.PI / 180.0)*(thetaX2 - thetaX1)/2));
			}
		} else {
			if(yAbove){
				x = -SENSOR_DISTANCE*Math.cos(((Math.PI / 180.0)*(thetaY2 - thetaY1)/2));
				y = SENSOR_DISTANCE*Math.cos(((Math.PI / 180.0)*(thetaX1)));
			} else {
				x = -SENSOR_DISTANCE*Math.cos(((Math.PI / 180.0)*(thetaY2 - thetaY1)/2));
				y = -SENSOR_DISTANCE*Math.cos(((Math.PI / 180.0)*(thetaX2 - thetaX1)/2));
			}
			
		}
		odo.setX(x);																				//Update x and y values
		odo.setY(y);
		nav.setXY(x, y);
		nav.travelTo(0, 0);																			//Travel to (0,0) and reset orientation
		nav.turnTo(-nav.getOrientation());
		nav.setOrientation((int)(odo.getTheta()*(180.0/Math.PI)));
		
	}
	@Override
	public double getFilteredData() {
		return lightPoller.filterData();
	}

}
