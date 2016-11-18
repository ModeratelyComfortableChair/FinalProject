package master.localization;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import master.Navigation;
import master.ScanQueue;
import master.odometry.Odometer;
import master.poller.USPoller;


/**
 * The USLocalizer performs localization for the EV3 Robot using an ultrasonic sensor.
 * With the robot placed at an unspecified angle within a corner tile, the USLocalizer
 * will rotate the EV3 robot until and latch the angles at which the sensor detects
 * the walls with respect to the FALLING_EDGE technique discussed in class. Afterwards,
 * it will the orientation to 0 degrees, and update the odometer.
 * 
 * @author Jerome Marfleet
 * @author Yu-Yueh Liu
 * @version 1.0
 * @since 2016-11-06
 *
 */
public class USLocalizer implements Localizer{

	//TODO: Change all values except TILE_WIDTH_CM
	public static final double LOW_ROTATION_SPEED = 70;				//Speed to rotate while distance < SPEED_BOUNDARY
	public static final double HIGH_ROTATION_SPEED = 70;			//Speed to rotate while distance > SPEED_BOUNDARY
	public static final double MAX_DISTANCE = 30;					//Distance at which to stop and latch angle
	public static final double SPEED_BOUNDARY = 10000;					//Distance at which to switch speeds
	public static final double MOMENT = 9.3;						//Distance between US sensor and center of rotation
	public static final double TILE_WIDTH_CM = 17;				//Width of distance	
	public static final double FILTER = 200;						//USPoller max readable distance
	
	private Odometer odo;
	private Navigation nav;
	private USPoller usPoller;
	
	private boolean lowSpeed;
	/**
	 * This constructs
	 * @param odo - The robot's odometer.
	 * @param usPoller - The ultrasonic poller corresponding to the sensor used for localization
	 * @param nav - The Navigation class is required to move the robot.
	 */
	public USLocalizer(Odometer odo, USPoller usPoller, Navigation nav) {
		this.odo = odo;
		this.nav = nav;
		this.usPoller = usPoller;
	}
	/**
	 * Performs the necessary angle localization according to the FALLING_EDGE angle correction.
	 * After, calls positionLocalization.
	 */
	@Override
	public void localize() {
		
		ScanQueue localQueue = new ScanQueue(7, MAX_DISTANCE);
		double angleA = 0;
		double angleB = 0;
		double theta;
		double edge;
		double lastEdge = 1000;
		usPoller.enable();
		getFilteredData();
		if(getFilteredData() < MAX_DISTANCE){							//If we start off facing the wall
				nav.rotateOnSpot((int) LOW_ROTATION_SPEED);						// keep rotating cw until the robot doesn't see the wall
				lowSpeed = true;
				edge = getFilteredData();

				while(edge < MAX_DISTANCE){
					if(edge > SPEED_BOUNDARY){
						if(lowSpeed){
							nav.rotateOnSpot((int) HIGH_ROTATION_SPEED);
							lowSpeed = false;
						}
					} else if(!lowSpeed){
						nav.rotateOnSpot((int) LOW_ROTATION_SPEED);
						lowSpeed = true;
					}
					edge = getFilteredData();
				}
				Sound.beep();
				nav.stopMotors();
				lastEdge = edge;
				localQueue.clearQueue();
				try {Thread.sleep(1000);} catch (InterruptedException e) {}
			}
			
			//Now we are definitely not facing the wall
			nav.rotateOnSpot((int) HIGH_ROTATION_SPEED);
			try {Thread.sleep(3000);} catch (InterruptedException e) {}
			edge = getFilteredData();

			while(edge > MAX_DISTANCE ){
				if(edge > SPEED_BOUNDARY){
					if(lowSpeed){
						nav.rotateOnSpot((int) HIGH_ROTATION_SPEED);
						lowSpeed = false;
					}
				} else if(!lowSpeed){
					nav.rotateOnSpot((int) LOW_ROTATION_SPEED);
					lowSpeed = true;
				}
				edge = getFilteredData();
			}																//Rotate cw until you face a wall
			Sound.beep();
			nav.stopMotors();
			lastEdge = edge;
			angleA = (180.0/Math.PI)*odo.getTheta();						//then latch the angle
			try {Thread.sleep(1000);} catch (InterruptedException e) {}
			
			nav.rotateOnSpot((int) -HIGH_ROTATION_SPEED);						// keep rotating ccw until the robot sees another wall
			try {Thread.sleep(3000);} catch (InterruptedException e) {}		//Initial delay so we don't detect the same angle
			edge = getFilteredData();
			while(edge > MAX_DISTANCE || edge > lastEdge){
				if(edge > SPEED_BOUNDARY){
					if(lowSpeed){
						nav.rotateOnSpot((int) -HIGH_ROTATION_SPEED);
						lowSpeed = false;
					}
				} else if(!lowSpeed){
					nav.rotateOnSpot((int) -LOW_ROTATION_SPEED);
					lowSpeed = true;
				}
				edge = getFilteredData();
			}
			Sound.beep();
			nav.stopMotors();
			lastEdge = edge;
			angleB =  (180.0/Math.PI)*odo.getTheta();						//Latch second angle
			try {Thread.sleep(1000);} catch (InterruptedException e) {}
			
			//Calculate orientation
			if(angleA < angleB){
				theta = (angleB - angleA)/2;
			} else {
				theta = (360 - angleA + angleB)/2;
			}		
			try {Thread.sleep(1000);} catch (InterruptedException e) {}
			//Localize robot with respect to x and y
			positionLocalization((int)theta);
			usPoller.disable();
	}
	
	/**
	 * positionLocalization is handles calculating the initial x and y position of the robot by
	 * measuring the distance of the robot and the walls of the corner. Because the orientation
	 * of the robot must be known beforehand, this method is set to private and only called on
	 * at the end of localize().
	 * 
	 * @param theta - the orientation of the robot after localize() is performed
	 */
	private void positionLocalization(int theta){
		double x, y;
		
		nav.turnTo(45 - theta);
		try {Thread.sleep(2000);} catch (InterruptedException e) {}				//Get X location with respect to wall
		x = MOMENT - TILE_WIDTH_CM + getFilteredData();
		
		nav.turnTo(-90);
		try {Thread.sleep(2000);} catch (InterruptedException e) {}				//Get Y location with respect to wall
		y = MOMENT - TILE_WIDTH_CM + getFilteredData();
		
		nav.turnTo(180);														//Face forward
		try {Thread.sleep(2000);} catch (InterruptedException e) {}
		
		odo.setPosition(new double [] {x, y, 0}, new boolean [] {true, true, true});		//Update nav and odo, positions.
		nav.getOdometerInfo();
		nav.travelTo(0,0);
		nav.turnTo(-nav.getOrientation());
	}
	
	/**
	 * Method acts a secondary filter for the USPoller, and is specific to the needs of the
	 * USLocalizer. In this case, the filter simply takes the minimum of the distance recorded,
	 * and our FILTER constant.
	 * 
	 * @return The filtered distance from the USPoller, as a double.
	 */
	@Override
	public double getFilteredData() {
		double distance = usPoller.filterData();
		return Math.min(distance, FILTER);
	}

}


