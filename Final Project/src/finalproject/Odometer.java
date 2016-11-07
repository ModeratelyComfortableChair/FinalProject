package finalproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
	private double x, y, theta, dY, dX, deltaD, deltaT;
	private int leftMotorTachoCount, rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double nowTachoL, nowTachoR, distL, distR;
	public double wheelRadius, wheelBase;
	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	/**
	 * Constructor
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param wheelRadius
	 * @param wheelBase
	 */
	public Odometer(EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor, double wheelRadius, double wheelBase) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		lock = new Object();
	}

	// run method (required for Thread)
	/**
	 * Method that extends from Thread class
	 * 
	 * Updates X, Y and Theta values of Odometer
	 */
	public void run() {
		long updateStart, updateEnd;

		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();

		while (true) {
			updateStart = System.currentTimeMillis();
			
			synchronized (lock) {

				nowTachoL = leftMotor.getTachoCount();
				nowTachoR = rightMotor.getTachoCount();
				distL = Math.PI*wheelRadius*(nowTachoL - leftMotorTachoCount) / 180;
				distR = Math.PI*wheelRadius*(nowTachoR - rightMotorTachoCount) / 180;
				leftMotorTachoCount = (int) nowTachoL;
				rightMotorTachoCount = (int) nowTachoR;
				deltaD = 0.5*(distL+distR);							// compute vehicle displacement
				deltaT = (distL-distR)/wheelBase;							// compute change in heading
				theta += deltaT;									// update heading
			    if(theta > 2*Math.PI){
			    	theta -= 2*Math.PI;
			    } else if (theta < 0){
			    	theta += 2*Math.PI;
			    }
				dX = deltaD * Math.sin(theta);						// compute X component of displacement
				dY = deltaD * Math.cos(theta);						// compute Y component of displacement
				x = x + dX;											// update estimates of X and Y position
				y = y + dY;				
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	
	/**
	 * This method gets the X, Y, Theta of the Robot and
	 * stores it in an array.
	 * 
	 * @param position - Array that will store X, Y, Theta
	 * @param update - Array of booleans
	 */
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}
	

	/**
	 * This method gets the X component
	 * 
	 * @return the X component of Odometer
	 */
	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	/**
	 * This method gets the Y component
	 * 
	 * @return the Y component of Odometer
	 */
	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	/**
	 * This method gets the Theta component
	 * 
	 * @return the Theta component of Odometer
	 */
	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	/**
	 * This method sets the X, Y, Theta of the Robot
	 * 
	 * @param position - Array containing updated values
	 * @param update - Array of booleans
	 */
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * This method sets the X component of the Odometer
	 * 
	 * @param x - X component of the Odometer
	 */
	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}
	
	/**
	 * This method sets the Y component of the Odometer
	 * 
	 * @param y - Y component of the Odometer
	 */
	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	/**
	 * This method sets the Theta component of the Odometer
	 * 
	 * @param theta - Theta component of the Odometer
	 */
	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	
	// accessors to motors
	/**
	 * This method returns both motors variables.
	 * 
	 * @return leftMotor and rightMotor variables
	 */
	public EV3LargeRegulatedMotor [] getMotors() {
		return new EV3LargeRegulatedMotor[] {this.leftMotor, this.rightMotor};
	}
	
	/**
	 * This method returns the leftMotor variable
	 * 
	 * @return leftMotor variable
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}
	
	/**
	 * This method returns the rightMotor variable
	 * 
	 * @return rightMotor variable
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}
	
	/**
	 * This method returns the leftMotorTachoCount
	 * 
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * This method sets the leftMotorTachoCount
	 * 
	 * @param leftMotorTachoCount - the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;	
		}
	}

	/**
	 * This method returns the rightMotorTachoCount
	 * 
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * This method sets the rightMotorTachoCount
	 * 
	 * @param rightMotorTachoCount - the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;	
		}
	}
}