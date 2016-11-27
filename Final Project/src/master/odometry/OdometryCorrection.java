/* 
 * OdometryCorrection.java
 */
package master.odometry;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.lcd.TextLCD;
/** 
 * OdometryCorrection handles correcting the position read by the 
 * Odometry using two Light sensors.
 * 
 * @author Jerome Marfleet
 * @author Yu-Yueh Liu
 * @author Brennan Laforge
 * @version 1.0
 * @since 2016-11-08
 *
 */
public class OdometryCorrection extends Thread {
	private EV3ColorSensor colorSensor;
	private SampleProvider colorProvider;
	private Odometer odometer;
	float[] colorSample;	
	
	private static final long CORRECTION_PERIOD = 10;
	private static final double LIGHT_SENSOR_MIN = 0.1;
	private static final double LIGHT_SENSOR_MAX = 0.4;
	private static final double SENSOR_OFFSET = 0.0;
	private static final double TILE_WIDTH = 30.48;
	private static final double BUFFER = 5.0;
	private static final double ANGLE_TOLERANCE = 19.0;
	private static final int MAX_LINE_COUNT = 10;

	/**
	 * Constructor
	 * 
	 * @param odometer Odometer Class
	 * @param LeftcolorSensor Left ColorSensor
	 * @param RightcolorSensor Right ColorSensor
	 */
	public OdometryCorrection(Odometer odometer, EV3ColorSensor colorSensor) {
		this.odometer = odometer;
		this.colorSensor = colorSensor;
	}

	/**
	 * Method that extends from Thread. 
	 * This is constantly running and correcting the odometer when
	 * the robot is travelling vertically or horizontally.
	 */
	public void run() {
		long correctionStart, correctionEnd;
		colorProvider = colorSensor.getRedMode();
		colorSample = new float[colorProvider.sampleSize()];
		double theta, newX, newY, distance;
		double oldX = 0.0;
		double oldY = 0.0;
		double xCorrection = 0.0;
		double yCorrection = 0.0;
		double correctionAngle = 0.0;
		int nX=0; 		
		int nY=0; 				
		
		// turn off when nav to identify blocks?
		while (true) {
			correctionStart = System.currentTimeMillis();
			colorProvider.fetchSample(colorSample, 0);		
			
			if (colorSample[0] <= LIGHT_SENSOR_MAX && colorSample[0] > LIGHT_SENSOR_MIN) {
				
				Sound.beep();				
				theta = Math.toDegrees(odometer.getTheta());
				newY = odometer.getY();
				newX = odometer.getX();
				distance = getDistance(newX, newY, oldX, oldY);
				
				nY = getYCount();
				nX = getXCount();
			
				double side = TILE_WIDTH-(2*BUFFER);
				double maxAngle = Math.toDegrees(Math.atan2(TILE_WIDTH, side));
				double maxDiagonal = Math.sqrt((TILE_WIDTH*TILE_WIDTH) + (side*side));
				if((distance > TILE_WIDTH && distance < maxDiagonal) && (yDir(theta) || xDir(theta))) {
					
					correctionAngle = Math.toDegrees(Math.acos(TILE_WIDTH/distance));
					
					if(xDir(theta)) { // traveling parallel to y axis
						if (newY > (BUFFER+(TILE_WIDTH*nY)) && newY < ((TILE_WIDTH-BUFFER)+(TILE_WIDTH*nY))) { // robot is not near black lines						
							
							// corrects theta value
							if(theta > 90 && theta < 90+maxAngle)
								odometer.setTheta(Math.toRadians(90+correctionAngle));
							else if(theta > 270 && theta < 270+maxAngle)
								odometer.setTheta(Math.toRadians(270+correctionAngle));
							else if(theta > 270-maxAngle && theta < 270)
								odometer.setTheta(Math.toRadians(270-correctionAngle));
							else if(theta > 90-maxAngle && theta < 90)
								odometer.setTheta(Math.toRadians(90-correctionAngle));
							
							xCorrection = SENSOR_OFFSET*Math.sin(Math.toRadians(correctionAngle));
							odometer.setX((TILE_WIDTH*nX)+ xCorrection); // x position correction						
							Sound.buzz();
						}
					}
					
					else if(yDir(theta)) { // traveling parallel to x axis
						if(newX > (BUFFER+(TILE_WIDTH*nX)) && newX < ((TILE_WIDTH-BUFFER)+(TILE_WIDTH*nX))) { // robot is not near black lines
							
							// corrects theta value
							if(theta < maxAngle)
								odometer.setTheta(Math.toRadians(correctionAngle));
							else if(theta > 180 && theta < 180+maxAngle)
								odometer.setTheta(Math.toRadians(180+correctionAngle));		
							else if(theta > 360-maxAngle)
								odometer.setTheta(Math.toRadians(360-correctionAngle));
							else if(theta > 180-maxAngle && theta < 180)
								odometer.setTheta(Math.toRadians(180-correctionAngle));
							
							yCorrection = SENSOR_OFFSET*Math.sin(Math.toRadians(correctionAngle));
							odometer.setY((TILE_WIDTH*nY)+ yCorrection); // y position correction
							Sound.buzz();
						}
					}
				}
				oldX = newX;
				oldY = newY;
			}								

			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					//nothing
				}
			}
		}
	}
	/**
	 * Determines if the robot travels in the positive Y direction
	 * @param theta the current angle of the robot
	 * @return true if the robot is traveling in the positive Y direction
	 */
	private boolean positiveYDir(double theta){
		if(theta > 360-ANGLE_TOLERANCE || theta < ANGLE_TOLERANCE){
			return true;
		}else
			return false;
	}
	
	/**
	 * Determines if the robot travels in the negative Y direction
	 * @param theta the current angle of the robot
	 * @return true if the robot is traveling in the negative Y direction
	 */
	private boolean negativeYDir(double theta) {
		if(theta > 180-ANGLE_TOLERANCE && theta < 180+ANGLE_TOLERANCE){	
			return true;
		}else
			return false;
	}
	/**
	 * Determines if the robot travels in Y direction
	 * @param theta the current angle of the robot
	 * @return true if the robot is travelling in Y direction
	 */
	private boolean yDir(double theta) {
		if(positiveYDir(theta) || negativeYDir(theta)) 
			return true;
		else
			return false;
	}
	/**
	 * Determines if the robot travels in the positive X direction
	 * @param theta the current angle of the robot
	 * @return true if the robot is traveling in the positive X direction
	 */
	private boolean positiveXDir(double theta){
		if(theta > 90-ANGLE_TOLERANCE && theta < 90+ANGLE_TOLERANCE){
			return true;
		}else
			return false;
	}
	/**
	 * Determines if the robot travels in the negative X direction
	 * @param theta the current angle of the robot
	 * @return true if the robot is traveling in the negative X direction
	 */
	private boolean negativeXDir(double theta) {
		if(theta > 270-ANGLE_TOLERANCE && theta < 270+ANGLE_TOLERANCE){	
			return true;
		}else
			return false;
	}
	/**
	 * Determines if the robot travels in X direction
	 * @param theta the current angle of the robot
	 * @return true if the robot is travelling in X direction
	 */
	private boolean xDir(double theta) {
		if(positiveXDir(theta) || negativeXDir(theta))
			return true;
		else
			return false;			
	}
	
	/**
	 * Simple function appiles pythagorean theorem for two points and returns the distance between them
	 * @param x1 initial x position
	 * @param y1 initial y position
	 * @param x2 desired x position
	 * @param y2 desired y position
	 * @return pythagorean theorem value for distance
	 */
	private double getDistance(double x1, double y1, double x2, double y2) {
		double diffX = x1-x2;
		double diffY = y1-y2; 
		return Math.sqrt((diffX * diffX) + (diffY * diffY));
	}
	
	/**
	 * Gets the nearest vertical line
	 * @return the number of horizontal lines the robot has theoretically crossed to get to its current position
	 */
	private int getYCount() {
		int nY = 0;
		double y = odometer.getY();
		for(int i = 0; i < MAX_LINE_COUNT; i++) {
			double upperBound = (i * TILE_WIDTH) + (TILE_WIDTH/2);
			double lowerBound = (i * TILE_WIDTH) - (TILE_WIDTH/2);
			if(y >= lowerBound && y <= upperBound) {
				nY = i;
				break;
			}
		}
		return nY;
	}
	
	/**
	 * Gets the nearest horizontal line
	 * @return the number of horizontal lines the robot has theoretically crossed to get to its current position
	 */
	private int getXCount() {
		int nX = 0;
		double x = odometer.getX();
		for(int i = 0; i < MAX_LINE_COUNT; i++) {
			double upperBound = (i * TILE_WIDTH) + (TILE_WIDTH/2);
			double lowerBound = (i * TILE_WIDTH) - (TILE_WIDTH/2);
			if(x >= lowerBound && x <= upperBound) {
				nX = i;
				break;
			}
		}
		return nX;
	}
}