/* DPM Team 12
 * 
 */
package master;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * AvoidObstacle will avoid the Obstacle once the thread is started
 * until the robot is safe.
 * 
 * @author Yu-Yueh Liu
 * @version 1.0
 * @since 2016-11-07
 *
 */

public class AvoidObstacle extends Thread {
	
	//Variable declarations/initializations
	private final int motorHigh = 200;
	private final int motorLow = 100;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	private Search searcher;

	private double WHEEL_RADIUS;
	private double TRACK;
	private boolean safe;
	private boolean rightSafe;
	private boolean leftSafe;
	
	//constructor
	/**
	 * Constructor
	 * @param searcher Search class
	 */
	public AvoidObstacle(Search searcher) {
		this.searcher = searcher;
		this.safe = false;
		this.leftMotor = searcher.leftMotor;
		this.rightMotor = searcher.rightMotor;
		this.WHEEL_RADIUS = searcher.WHEEL_RADIUS;
		this.TRACK = searcher.TRACK;
		this.rightSafe=true;
		this.leftSafe=true;
	}
	
	//obstacle avoidance - get around the blocks
	/**
	 * Method that extends from Thread
	 * This method will run continuously until the robot is safe
	 */
	public void run(){
		while(!safe){
			if(rightSafe){
				turn(90);
				if(clearPath()){
					forward(20);
					turn(-90);
					if(clearPath()){
						forward(30);
						safe=true;
					}
				}else{
					turn(-90);
					rightSafe=false;
				}
			}else if(leftSafe){
				turn(-90);
				if(clearPath()){
					forward(20);
					turn(90);
					if(clearPath()){
						forward(30);
						safe=true;
					}
				}else{
					turn(90);
					leftSafe=false;
				}				
			}else{
				turn(180);
				forward(30);
				safe=true;
			}
		}
	}

//	//check if robot is 90 degrees past the initial angle
//	//it would mean that the robot made its way around the obstacle
//	public boolean isSafe(){
//		if(true){ //some condition to check if robot is safe
//			return true;
//		}else 
//			return false;
//	}
	
	//returns boolean safe
	public boolean resolved(){
		return safe;
	}
	
	public void forward(int dist){
		leftMotor.setSpeed(150+1);
		rightMotor.setSpeed(150);

		leftMotor.rotate(convertDistance(WHEEL_RADIUS, dist), true);
		rightMotor.rotate(convertDistance(WHEEL_RADIUS, dist), false);
	}

	public boolean clearPath(){
		if(searcher.readUSDistance()<35){
			return false;
		}else
			return true;
	}
	
	//Turning method
	public void turn(double theta){
		//LCD.drawString("Turn: "+Double.toString(theta), 0, 7);
		leftMotor.setSpeed(50);
		rightMotor.setSpeed(50);
		leftMotor.rotate(convertAngle(2.125, 15.6, theta), true);
		rightMotor.rotate(-convertAngle(2.125, 15.6, theta), false);
	}
	
	//convert distance to the angle of rotation of the wheels in degrees
	public int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	//convert cart angle to the angle of rotation of the wheels in degrees
	public int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}
