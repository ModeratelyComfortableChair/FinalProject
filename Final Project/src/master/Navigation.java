


package master;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import master.odometry.Odometer;

/**
 * Navigation handles all movement of the Robot's wheel motors. No class can move the robot
 * unless it access the methods through the nav class.
 * 
 * @author Jerome Marfleet
 * @author Yu-Yueh Liu
 * @version 1.0
 * @since 2016-11-06
 *
 */
public class Navigation {

	private static final int FORWARD_SPEED = 70;
	private static final int ROTATE_SPEED = 150;
	private int orientation = 0;
	private double[] lastPos = new double[] {0,0};

	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private Odometer odo;
	private double leftRadius, rightRadius, width;
	private boolean alert, turning, control, travelling, allowAlert, avoiding;

	/**
	 * Constructor
	 * @param odo - Robot's Odometer
	 */
	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
		//get Odometer
		this.odo = odo;

		//get Motors
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		//get parameters
		this.leftRadius = odo.wheelRadius;
		this.rightRadius = odo.wheelRadius;
		this.width = odo.wheelBase;
	}

	/**
	 * Sets both wheels to FORWARD_SPEED and drives forward
	 */
	public void driveForward(){
		leftMotor.setSpeed(FORWARD_SPEED);									//Set Wheel speeds
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();
	}

	/**
	 * Sets both wheels to BACKWARD_SPEED and drives backward
	 */
	public void driveBackward(){
		leftMotor.setSpeed(FORWARD_SPEED);									//Set Wheel speeds
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.backward();
		rightMotor.backward();
	}
	
	public void driveDistanceForward(double distance){
		leftMotor.setSpeed(FORWARD_SPEED);									//Set Wheel speeds
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(leftRadius, distance), true);		//Rotate Wheels
		rightMotor.rotate(convertDistance(rightRadius, distance), false);
	}

	/**
	 * travelTo takes in a desired x and y position, and Moves the robot to that position
	 * 
	 * @param x - desired x position
	 * @param y - desired y position
	 */
	public void travelTo(double x, double y){
		if(alert){
			return;
		}
		travelling = true;
		turnTo(getAngle(lastPos[0], lastPos[1], x, y, orientation));		//Face direction
		double distance = getDistance(lastPos[0], lastPos[1], x, y);		//Obtain traveling distance
		leftMotor.setSpeed(FORWARD_SPEED);									//Set Wheel speeds
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(leftRadius, distance), true);		//Rotate Wheels
		rightMotor.rotate(convertDistance(rightRadius, distance), true);

		while(leftMotor.isMoving()){
			if(alert){
				stopMotors();
				travelling = false;
				return;
			}
		}

		lastPos[0] = x;														//Update new position
		lastPos[1] = y;
		travelling = false;
	}

	/**
	 * Rotates the robot about a given angle, theta
	 * 
	 * @param theta - angle of turn
	 */
	public void turnTo(double theta){								
		turning = true;
		leftMotor.setSpeed(ROTATE_SPEED);									//Rotate one spot
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(leftRadius, width, theta), true);
		rightMotor.rotate(-convertAngle(rightRadius, width, theta), false);
		orientation += theta;			
		turning = false;													//Adjust orientation

	}

	/**
	 * Takes a radius and distance double, and returns the needed angle of rotation
	 * for the wheel motors in order for the robot to reach given distance.
	 * 
	 * @param radius - radius of robot tire
	 * @param distance - wheelbase width of the robot
	 * @return - integer value representing angle of tire rotation
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	//increasing radius reduces distance and turning angle
	//increasing width increases travelling,
	//if i want to increase the distance, i need to decrease the radius
	//If i want to increase the turning angle, i need to decrease the radius or increase the width
	/**
	 * Takes a radius, a distance, and a turning angle, and invokes convertDistance 
	 * with the desired distance to turn the wheels in order to turn the robot the given angle
	 * 
	 * @param radius - radius of robot tire
	 * @param width - Wheel-base width of the robot
	 * @param angle - desired rotation of the robot
	 * @return a call for convertDistance, which returns an integer
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * getAngle takes in the current x-y position and orientation of the robot, and the
	 * x-y position of a desired destination. The method returns the minimum turning angle
	 * required for the robot to face the new position
	 * 
	 * @param x1 - Initial x position
	 * @param y1 - Initial y position
	 * @param x2 - Final x position
	 * @param y2 - Final y position
	 * @param orientation - current angle of the robot
	 * @return integer of the angle needed to rotate in order to face new position
	 */

	public int getAngle(double x1, double y1, double x2, double y2, int orientation){
		double diffX = x2 - x1;
		double diffY = y2 - y1;
		int angle = 0;
		if(diffY == 0){
			if(diffX > 0){
				angle = 90;
			} else {
				angle = -90;
			}
		} else if (diffX == 0){
			if(diffY > 0){
				angle = 0;
			} else {
				angle = 180;
			}
		} else {
			if(diffX > 0 && diffY > 0){
				angle = (int) (Math.atan(diffX/diffY) *180/Math.PI);

			} else if (diffX < 0 && diffY < 0){
				angle = 180 + (int) (Math.atan(diffX/diffY) *180/Math.PI);

			} else if (diffY < 0){
				angle = 90 - (int) (Math.atan(diffY/diffX) *180/Math.PI);

			} else if (diffX < 0){
				angle = 270 - (int) (Math.atan(diffY/diffX) *180/Math.PI);

			}
		}
		angle -= orientation;
		if(Math.abs((double)angle) > 180){
			if(angle < 0){
				angle += 360;
			} else {
				angle = 360 - angle;
			}
		}
		return angle;

	}

	//
	private static double getDistance(double x1, double y1, double x2, double y2){
		double diffX = x2 - x1;
		double diffY = y2 - y1;

		return Math.sqrt(diffX*diffX + diffY*diffY);
	}

	/**
	 * Setter method for alert property
	 * @param alert - boolean to be set as new alert
	 */
	public void setAlert( boolean alert ){
		this.alert = alert;
	}

	/**
	 * Getter method for alert property
	 * @return alert boolean
	 */
	public boolean getAlert(){
		return alert;
	}

	/**
	 * Setter method for control property
	 * @param control - boolean to be set as new control
	 */
	public void setControl(boolean control){
		this.control = control;
	}

	/**
	 * Getter method for control property
	 * @return control boolean
	 */
	public boolean getControl(){
		return control;
	}

	/**
	 * Setter method for turning property
	 * @param turning - boolean to be set as new turning
	 */
	public void setTurning(boolean turning) {
		this.turning = turning;
	}

	/**
	 * Getter method for control property
	 * @return control boolean
	 */
	public boolean isTurning() {
		return turning;
	}

	
	/**
	 * Setter method for the traveling property
	 * @param traveling - boolean to be new traveling
	 */
	public void setTraveling(boolean traveling) {
		this.travelling = traveling;
	}


	/**
	 * Getter method for the traveling property
	 * @return travelling boolean
	 */
	public boolean isTravelling() {
		return travelling;
	}


	/**
	 * Rotates the robot on spot at a given speed
	 * @param rotateSpeed - speed for wheels to rotate 
	 */
	public void rotateOnSpot(int rotateSpeed){
		int rotateSpeedAbs = Math.abs(rotateSpeed);
		leftMotor.setSpeed(rotateSpeedAbs);									
		rightMotor.setSpeed(rotateSpeedAbs);
		if(rotateSpeed > 0){
			leftMotor.forward();
			rightMotor.backward();
		} else if (rotateSpeed < 0){
			leftMotor.backward();
			rightMotor.forward();
		} else {
			return;
		}
		return;
	}

	/**
	 * Shuts off both motors. Will not return until motors have stopped.
	 */
	public void stopMotors(){
		leftMotor.stop(true);
		rightMotor.stop();
		return;
	}

	/**
	 * Setter method for the x and y position
	 * @param x - new x position
	 * @param y - new y position
	 */
	public void setXY(double x, double y){
		lastPos[0] = x;
		lastPos[1] = y;
	}

	/**
	 * Setter method for the orientation (Angle) of the robot
	 * @param orientation - integer to be set as new orietation
	 */
	public void setOrientation(int orientation){
		this.orientation = orientation;
	}

	/**
	 * Getter method for the orientation integer
	 * @return the orientation integer
	 */
	public int getOrientation(){
		return orientation;
	}

	/**
	 * Getter method for the allowAlert boolean
	 * @return the allowAlert method
	 */
	public boolean allowAlert() {
		return allowAlert;
	}

	/**
	 * updates the x, y, and orientation values of Navigation with the odometer values.
	 */
	public void getOdometerInfo(){
		setXY(odo.getX(), odo.getY());
		setOrientation((int)((180 / Math.PI)*odo.getTheta()));
	}

	/**
	 * Getter method for the avoiding boolean
	 * @return the avoiding boolean
	 */
	public boolean isAvoiding() {
		return avoiding;
	}

	/**
	 * Setter Method for the avoiding boolean
	 * @param avoiding - the avoiding boolean
	 */
	public void setAvoiding(boolean avoiding) {
		this.avoiding = avoiding;
	}

	/**
	 * Returns the orientation of the robot in terms of a String.
	 * @return String representation of the orientation
	 */
	public String getHeading(){
		if((orientation >=0 && orientation <= 10) || (orientation <= 360 && orientation >= 350)){
			return "POS_Y";
		} else if (orientation >= 80 && orientation <= 100){
			return "POS_X";
		} else if (orientation >= 170 && orientation <= 190){
			return "NEG_Y";
		} else if (orientation >= 260 && orientation <= 280){
			return "NEG_X";
		} else {
			return "IDK";
		}
	}
	
	//rotate method
	public void rotate(double theta){
		leftMotor.rotate(convertAngle(leftRadius, width, theta), true);
		rightMotor.rotate(-convertAngle(leftRadius, width, theta), false);
	}
}
