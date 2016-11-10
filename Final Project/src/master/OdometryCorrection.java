/* 
 * OdometryCorrection.java
 */
package master;

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
 * @version 1.0
 * @since 2016-11-08
 *
 */
public class OdometryCorrection extends Thread {
	private EV3ColorSensor RcolorSensor;
	private EV3ColorSensor LcolorSensor;
	private SampleProvider colorProvider;
	float[] colorSample;
	
	
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	// constructor
	/**
	 * Constructor
	 * 
	 * @param odometer Odometer Class
	 * @param LeftcolorSensor Left ColorSensor
	 * @param RightcolorSensor Right ColorSensor
	 */
	public OdometryCorrection(Odometer odometer, EV3ColorSensor LeftcolorSensor, EV3ColorSensor RightcolorSensor) {
		this.odometer = odometer;
		this.RcolorSensor = RightcolorSensor;
		this.LcolorSensor = LeftcolorSensor;
	}

	// run method (required for Thread)
	/**
	 * Method that extends from Thread. 
	 * This is constantly running and correcting the odometer when
	 * the robot is travelling vertically or horizontally.
	 */
	public void run() {
		long correctionStart, correctionEnd;
		colorProvider = RcolorSensor.getRedMode();
		colorSample = new float[colorProvider.sampleSize()];
		double theta, offset, x, y;
		int nX=0; 						//measures the no. of lines crossed in X
		int nY=0; 						//measures the no. of lines crossed in Y
		offset = 0;
				
		while (true) {
			correctionStart = System.currentTimeMillis();
			colorProvider.fetchSample(colorSample, 0);
									
			// put your correction code here
			if (colorSample[0] != -0.2){ //tiles
				//nothing
			}
			if (colorSample[0] <= 0.4 && colorSample[0] > 0.1){ // line detected 
				theta = odometer.getTheta();
				y = odometer.getY();
				x = odometer.getX();
				
				if(Ydir(theta)){
					nY++;
					System.out.println("           nY= " + nY);
				}else if(Xdir(theta)){
					nX++;	
					System.out.println("           nX= " + nX);
				}
				
				int [] a = {4, 25, 500, 7000, 5};
				
				Sound.playNote(a, 988, 100);
				Sound.playNote(a, 1319, 300);
				
				//The correction method below was adjusted to correct the imprecision of the odometer including slight
				//modifications for the distances of tiles
				
				//if moving in Y-direction
				if(Ydir(theta)){
					for(int i = 1; i<=nY; i++){
						if (y > (14.24*i-5) && y < (14.24*i+5)){ //i used 14.24 because it's closer to 13 than 15.25
							odometer.setY(13*i);		// correct y value to the nearest multiple of 13 (by trial & error)
							break;
						}
					}
				//if moving in the X-direction
				}else if(Xdir(theta)){ 
					for(int j = 1; j<=nX;j++){
						if (x > 14.24*j-5 && x <(14.24*j+5)){ 
							odometer.setX(13*j);		// correct x value to the nearest multiple of 13 (by trial & error)
							break;
						}			
					}
				}				
				
			}

			// this ensures the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
	
	/**
	 * Determines if the robot travels in Y direction
	 * @param theta the current angle of the robot
	 * @return true if the robot is travelling in Y direction
	 */
	public boolean Ydir(double theta){
		if((theta > 5.5 || theta < 0.78) || (theta > 2.35 && theta < 3.92)){	//Returns true if travelling in Y-direction
			return true;
		}else
			return false;
	}
	
	/**
	 * Determines if the robot travels in X direction
	 * @param theta the current angle of the robot
	 * @return true if the robot is travelling in X direction
	 */
	public boolean Xdir(double theta){
		if((theta > 0.78 && theta < 2.35) || (theta > 3.92 && theta < 5.5)){	//Returns true if travelling in X-direction
			return true;
		}else
			return false;
	}
}