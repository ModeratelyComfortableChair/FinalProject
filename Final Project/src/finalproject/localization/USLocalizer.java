package finalproject.localization;

import finalproject.Navigation;
import finalproject.Odometer;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class USLocalizer implements Localizer{

	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static final double LOW_ROTATION_SPEED = 60;				//Speed to rotate while distance < SPEED_BOUNDARY
	public static final double HIGH_ROTATION_SPEED = 100;			//Speed to rotate while distance > SPEED_BOUNDARY
	public static final double MAX_DISTANCE = 40;					//Distance at which to stop and latch angle
	public static final double SPEED_BOUNDARY = 60;					//Distance at which to switch speeds
	public static final double MOMENT = 6.5;						//Distance between US sensor and center of rotation
	public static final double TILE_WIDTH = 30.48;			
	
	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	private Navigation nav;
	public USLocalizer(Odometer odo,  SampleProvider usSensor, float[] usData,
			LocalizationType locType, Navigation nav) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		this.nav = nav;
	}
	
	@Override
	public void localize() {
		double [] pos = new double [3];
		double angleA = 0;
		double angleB = 0;
		double theta;
		double x;
		double y;
		float lastEdge = 1000;
		float edge;
		try {Thread.sleep(1000);} catch (InterruptedException e) {}
		if(getFilteredData() < MAX_DISTANCE){							//If we start off facing the wall
				nav.rotateOnSpot((int) HIGH_ROTATION_SPEED);						// keep rotating cw until the robot doesn't see the wall
				edge = getFilteredData();
				while(edge < MAX_DISTANCE){
					edge = getFilteredData();
					if(edge > SPEED_BOUNDARY){
						nav.rotateOnSpot((int) HIGH_ROTATION_SPEED);
					} else {
						nav.rotateOnSpot((int) LOW_ROTATION_SPEED);
					}
				}
				Sound.beep();
				nav.stopMotors();
				lastEdge = edge;
				try {Thread.sleep(1000);} catch (InterruptedException e) {}
			}
			
			//Now we are definitely not facing the wall
			nav.rotateOnSpot((int) HIGH_ROTATION_SPEED);
			edge = getFilteredData();
			while(edge > MAX_DISTANCE ){
				if(edge > SPEED_BOUNDARY){
					nav.rotateOnSpot((int) HIGH_ROTATION_SPEED);
				} else {
					nav.rotateOnSpot((int) LOW_ROTATION_SPEED);
				}
				edge = getFilteredData();
			}																//Rotate cw until you face a wall
			Sound.beep();
			nav.stopMotors();
			lastEdge = edge;
			angleA = (180.0/Math.PI)*odo.getTheta();						//then latch the angle
			try {Thread.sleep(3000);} catch (InterruptedException e) {}
			
			nav.rotateOnSpot((int) -HIGH_ROTATION_SPEED);						// keep rotating ccw until the robot sees another wall
			try {Thread.sleep(1000);} catch (InterruptedException e) {}		//Initial delay so we don't detect the same angle
			edge = getFilteredData();
			while(edge > MAX_DISTANCE || edge > lastEdge){
				if(edge > SPEED_BOUNDARY){
					nav.rotateOnSpot((int) HIGH_ROTATION_SPEED);
				} else {
					nav.rotateOnSpot((int) LOW_ROTATION_SPEED);
				}
				edge = getFilteredData();
			}
			Sound.beep();
			nav.stopMotors();
			lastEdge = edge;
			angleB =  (180.0/Math.PI)*odo.getTheta();						//Latch second angle
			try {Thread.sleep(1000);} catch (InterruptedException e) {}
			
			if(angleA < angleB){
				theta = (angleB - angleA)/2;
			} else {
				theta = (360 - angleA + angleB)/2;
			}
			nav.turnTo(135 - theta);										//Calculate orientation, and turn robot until we face 0 degrees
			try {Thread.sleep(1000);} catch (InterruptedException e) {}
			
		 
	}
	public float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = Math.abs(usData[0]*100);
				
		return Math.min(distance, 200);
	}

		
	

}
