


package finalproject;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
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
	

	public Navigation(Odometer odo){
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		this.leftRadius = odo.wheelRadius;
		this.rightRadius = odo.wheelRadius;
		this.width = odo.wheelBase;
		this.odo = odo;
	}

	
	public void driveForward(){
		leftMotor.setSpeed(FORWARD_SPEED);									//Set Wheel speeds
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();
	}
	public void driveBackward(){
		leftMotor.setSpeed(FORWARD_SPEED);									//Set Wheel speeds
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.backward();
		rightMotor.backward();
	}
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
	public void turnTo(double theta){								
		turning = true;
		leftMotor.setSpeed(ROTATE_SPEED);									//Rotate one spot
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(leftRadius, width, theta), true);
		rightMotor.rotate(-convertAngle(rightRadius, width, theta), false);
		orientation += theta;			
		turning = false;													//Adjust orientation

	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private static int getAngle(double x1, double y1, double x2, double y2, int orientation){
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
	private static double getDistance(double x1, double y1, double x2, double y2){
		double diffX = x2 - x1;
		double diffY = y2 - y1;

		return Math.sqrt(diffX*diffX + diffY*diffY);
	}
	public void setAlert( boolean alert ){
		this.alert = alert;
	}
	public boolean getAlert(){
		return alert;
	}
	public boolean getControl(){
		return control;
	}
	public void setControl(boolean control){
		this.control = control;
	}


	public boolean isTurning() {
		return turning;
	}


	public void setTurning(boolean turning) {
		this.turning = turning;
	}

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

	public void stopMotors(){
		leftMotor.stop(true);
		rightMotor.stop();
		return;
	}

	public void setXY(double x, double y){
		lastPos[0] = x;
		lastPos[1] = y;
	}
	public void setOrientation(int orientation){
		this.orientation = orientation;
	}
	public int getOrientation(){
		return orientation;
	}

	public boolean isTravelling() {
		return travelling;
	}

	public void setTravelling(boolean travelling) {
		this.travelling = travelling;
	}

	public boolean allowAlert() {
		return allowAlert;
	}
	public void getOdometerInfo(){
		setXY(odo.getX(), odo.getY());
		setOrientation((int)((180 / Math.PI)*odo.getTheta()));
	}

	public boolean isAvoiding() {
		return avoiding;
	}

	public void setAvoiding(boolean avoiding) {
		this.avoiding = avoiding;
	}
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
}
