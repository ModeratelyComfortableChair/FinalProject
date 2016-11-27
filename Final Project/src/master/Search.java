/* DPM Team 12
 * 
 */
package master;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import master.localization.LocalizationMaster;
import master.odometry.Odometer;
import master.poller.USPoller;
import master.poller.UltrasonicController;

/**
 * Search will constantly search once the robot finishes Localizing to (0,0)
 * 
 * @author Yu-Yueh Liu
 * @version 1.0
 * @since 2016-11-07
 *
 */
public class Search extends Thread implements UltrasonicController{
	// Constants for navigation 
	private double xCurrent;
	private double yCurrent;
	public double thetaCurrent;
	private double xDest;
	private double yDest;
	public double thetaDest;
	public double WHEEL_RADIUS;
	public double TRACK;
	private static final int FORWARD_SPEED = 150, ROTATE_SPEED = 100, ACCELERATION = 1000, SCAN_SPEED = 20, SCAN_SIZE = 25, ID_SIZE = 20, ID_DISTANCE = 6, IGNORE_TIME = 3000;
	private static final double SCAN_RADIUS = 60.96, ID_RADIUS = 15;
	private static final double SAFE_DETECTION = 40;
	private static final double ID_TIME = 3000;
	//other rotation speed
	private static int RSPEED = 50, counter=0;

	// Array that determines instrument: Piano
	public int [] a = {4, 25, 500, 7000, 5};	

	// Constants for USPoller
	//private double distance;
	private final double objectRange = 7.3; 

	//
	public double distObject, xObject, yObject;
	private double distance, rangeObject = 50;

	//Variable for LightSensor
	private float[] colorData;
	private SampleProvider colorProvider;

	// Boolean variables
	private boolean isNavigating = false;
	private boolean isObject = false;
	public boolean isChecked = false;
	private boolean isBlock = false;
	public boolean isAvoiding = false;
	public boolean isDetected;
	public boolean notScanning = true;

	// Initialization of some variables
	public EV3LargeRegulatedMotor leftMotor, rightMotor, lift1, lift2; 	//They are public because AvoidObstacle calls them
	public RegulatedMotor claw;
	public Odometer odo;
	private Navigation nav;
	private LocalizationMaster localization;
	public USPoller usLower;
	public USPoller usUpper;
	private double scanStartAngle;
	private int[] corner = new int[2];
	private int[] goodZone = new int[4];
	private int[] badZone = new int[4];
	private boolean scanned=false;
	private int blockCount=0;
	private int stage = 2;
	private int cornerNum;
	private Tile scanEnd;
	AvoidObstacle avoider;


	// Enum declaration/initialization
	enum State {INIT, SCAN, TRAVELLING};


	// Navigation Constructor
	/**
	 * Constructor 
	 * 
	 * @param odometer Odometer Class
	 * @param nav Navigation Class
	 * @param turner
	 * @param hook
	 * @param colorSensor Color Sensor Object
	 * @param colorData Data from Color Sensor
	 * @param localization Localization Class
	 */
	public Search(Odometer odometer, Navigation nav, EV3LargeRegulatedMotor lift1, EV3LargeRegulatedMotor lift2,
			RegulatedMotor claw, LocalizationMaster localization, USPoller usLower, USPoller usHigher) {
		//Set variables
		this.xCurrent = 0;
		this.yCurrent = 0;
		this.thetaCurrent = 0;
		this.odo = odometer;
		this.nav = nav;
		this.WHEEL_RADIUS = odometer.wheelRadius;
		this.TRACK = odometer.wheelBase;
		this.usLower = usLower;
		this.usUpper = usHigher;

		//Get motors
		EV3LargeRegulatedMotor[] motors = this.odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		this.lift1 = lift1;
		this.lift2 = lift2;
		this.claw = claw;

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);

		//Localization
		this.localization = localization;
		avoider = new AvoidObstacle(nav, usUpper, odo);

	}




	// Run method
	/**
	 * Method that extends from Thread
	 * This method contains the main State Machine
	 * that handles the search for styrofoam blocks
	 */
	public void run(){
		
		State state = State.INIT;
		double angle = 0;
		Tile currentTile = null;

		while(true){
			switch(state){
			case INIT:
				LCD.drawString("            ", 0, 5);
				LCD.drawString("INIT", 0, 5);

				odo.getPosition(new double[]{0, 0, 0}, new boolean[]{true, true, true});
				nav.getOdometerInfo();
				localization.localize();

				if(corner[0] == 0){
					if(corner[1] == 0){
						angle = 0;
						cornerNum = 1;
						currentTile = new Tile(1, 1);
					} else {
						angle = 90;
						cornerNum = 4;
						currentTile = new Tile(1, 14);
					}
				}else{
					if(corner[1] == 0){
						angle = 270;
						cornerNum = 2;
						currentTile = new Tile(14, 1);
					} else {
						angle = 180;
						cornerNum = 3;
						currentTile = new Tile(14, 14);
					}
				}
				scanEnd = currentTile;
				odo.setPosition(new double[]{corner[0],corner[1], angle}, new boolean[]{true, true, true});

				nav.getOdometerInfo();
				Victory(a);
				LCD.drawString("Corner x: " + Integer.toString(corner[0]), 0, 6);
				LCD.drawString("Corner y: " + Integer.toString(corner[1]), 0, 7);
				nav.travelTo(currentTile.getCenterX(), currentTile.getCenterY(), false);
				nav.turnTo(angle - nav.getOrientation());
				state = State.SCAN;
				break;

			case SCAN:
				LCD.drawString("            ", 0, 5);
				LCD.drawString("SCAN", 0, 5);
				usLower.enable();
				scanStartAngle = (odo.getTheta())*(180.0/Math.PI);
				ScanQueue scanQueue = new ScanQueue(SCAN_SIZE, SCAN_RADIUS);
				ScanQueue idQueue = new ScanQueue(ID_SIZE, ID_RADIUS);
				double distance, startTime, currentTime, lastAngle;
				boolean block = true;
				boolean moved;
				double[] idPosition = {0, 0, 0};
				while(getAngleDiffCW( scanStartAngle, odo.getTheta()*(180.0/Math.PI)) < 90){
					nav.rotateOnSpot(SCAN_SPEED);
					distance = usLower.filterData();
					moved = false;
					if(block){
						if(scanQueue.checkAndAddUnder(distance)){
							nav.stopMotors();
							coinSound();
							double average = scanQueue.getAverage();
							if(average < SAFE_DETECTION && getAngleDiffCW( scanStartAngle, odo.getTheta()*(180.0/Math.PI)) > 30){
								nav.turnTo(20);
							}
							double closingDistance = average - ID_DISTANCE;
							lastAngle = odo.getTheta() * (180.0 / Math.PI);
							if(closingDistance > 0){
								nav.setForwardSpeed(300);
								nav.driveDistanceForward(closingDistance, false);
								moved = true;
								nav.setForwardSpeed(70);
							}
							scanQueue.clearQueue();
							scanCorrect();
							usLower.disable();
							Delay.msDelay(500);
							//Pull and update values for ID_TIME. Update block boolean if we detect it
							usUpper.enable();

							startTime = System.currentTimeMillis();
							currentTime = System.currentTimeMillis();
							while(currentTime - startTime < ID_TIME){
								distance = usUpper.filterData();
								if(idQueue.checkAndAddUnder(distance)){
									block = false;
									Sound.beep();
								}
								currentTime = System.currentTimeMillis();
							}
							usUpper.disable();
							if(block){
								Sound.playNote(a, 440, 250);
								moved = false;
								blockCatch();
								blockCount++;
								nav.getOdometerInfo();
								nav.setForwardSpeed(300);
								if(blockCount == 2){
									state=State.TRAVELLING;
									scanEnd = new Tile(findTile(odo.getX()), findTile(odo.getY()));
									travelTo(scanEnd.getCenterX(), scanEnd.getCenterY());
									break;
								}
							} else {
								Sound.beep();
								nav.driveDistanceBackward(10, false);
							}
							if(moved){
								odo.getPosition(idPosition, new boolean[]{true, true, true});
								nav.getOdometerInfo();
								nav.setForwardSpeed(300);
								nav.travelTo(currentTile.getCenterX(), currentTile.getCenterY(), false);
								nav.setForwardSpeed(70);
								nav.turnTo(lastAngle - nav.getOrientation());
							}
							idQueue.clearQueue();
							usLower.enable();
							moved = false;


						}
					} else {	//Timed Ignorance of wooden block
						startTime = System.currentTimeMillis();
						currentTime = System.currentTimeMillis();
						while(currentTime - startTime < IGNORE_TIME){
							currentTime = System.currentTimeMillis();
							System.out.print(" Ignoring ");
						}
						block = true;
					}

				}
				usLower.disable();
				nav.stopMotors();
				nav.getOdometerInfo();
				state=State.TRAVELLING;
				break;

			case TRAVELLING:								//TRAVELLING STATE

				LCD.drawString("            ", 0, 5);
				LCD.drawString("TRAVELLING", 0, 5);
				//Travel between seconds scan zone and collection zone
				Tile collection = new Tile(findTile(goodZone[0]), findTile(goodZone[1]));
				nav.getOdometerInfo();
				Tile endZone = avoider.findPath(collection.getX(), collection.getY(), scanEnd.getX(), scanEnd.getY());
				//If the tile we wish to drop off at is clear.
				if(collection.getX() != endZone.getX() || collection.getX() != collection.getY()){
					collection = new Tile(findTile(goodZone[0] + 30), findTile(goodZone[0]));
					endZone = avoider.findPath(collection.getX(), collection.getY(), endZone.getX(), endZone.getY());
				}
				nav.turnTo(nav.getAngle(odo.getX(), odo.getY(), collection.getCenterX()-30, collection.getCenterY(), nav.getOrientation()));
				liftDownToDrop();
				liftUp();
				nav.getOdometerInfo();
				avoider.findPath(currentTile.getX(), currentTile.getY(), collection.getX(), collection.getY());
				nav.travelTo(corner[0], corner[1], false);
				nav.turnTo(angle - nav.getOrientation());
				System.exit(0);

				break;
			}
			try{											//Thread to sleep for 30miliseconds
				Thread.sleep(30);
			}catch(InterruptedException e){					//catch exception if current thread is interrupted
				e.printStackTrace();
			}
		}
	}

	private int findTile(double coord) {
		return (int)(coord/30 + 1);
	}

	public boolean isObject() {
		return isObject;
	}

	public boolean notScanning(){
		return this.notScanning;
	}

	public boolean isDetected(){
		return this.isDetected;
	}


	public void blockCatch(){
		liftDownToGrab();
		int turnAngle=20;
//		int distForward=20;
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, turnAngle), true);
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, turnAngle), false);
		leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, turnAngle), true);
		rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, turnAngle), false);
		nav.driveDistanceForward(10, false);
		grab();
		liftUp();
	}


	public void liftDownToGrab() {
		lift1.setAcceleration(500);
		lift2.setAcceleration(500);
		lift1.setSpeed(100);
		lift2.setSpeed(100);
		//		lift1.rotate(-1470,true);
		//		lift2.rotate(-1470,false);
		lift1.rotate(-1020,true);
		lift2.rotate(-1020,false);
		claw.setAcceleration(80);
		claw.setSpeed(50);
		claw.rotateTo(60);
		lift1.rotate(-550,true);
		lift2.rotate(-550,false);
	}
	public void liftDownToDrop(){
		lift1.setAcceleration(500);
		lift2.setAcceleration(500);
		lift1.setSpeed(100);
		lift2.setSpeed(100);
		lift1.rotate(-1570,true);
		lift2.rotate(-1570,false);
		claw.setAcceleration(80);
		claw.setSpeed(50);
		claw.rotateTo(60);
		//		lift1.rotate(-1470,true);
		//		lift2.rotate(-1470,false);
	}

	public void liftUp(){
		lift1.setAcceleration(500);
		lift2.setAcceleration(500);
		lift1.setSpeed(100);
		lift2.setSpeed(100);
		lift1.rotate(1570,true);
		lift2.rotate(1570,false);
	}

	public void grab() {
		claw.setAcceleration(100);
		claw.setSpeed(100);
		claw.rotateTo(0);
	}

	// Return Object's X-coordinate
	public double targetX(double dist){
		double Theta = odo.getTheta();								// Angle of robot is facing:
		if(Theta>=0 && Theta<=90){										// Top right quadrant
			xObject=odo.getX()+(Math.sin(Math.toRadians(Theta))*dist);
		}else if(Theta>90 && Theta <=180){								// Bottom right quadrant
			xObject=odo.getX()+(Math.sin(Math.toRadians(90-(Theta-90)))*dist);
		}else if(Theta>180 && Theta <=270){								// Bottom left quadrant
			xObject=odo.getX()-(Math.sin(Math.toRadians(Theta-180))*dist);
		}else if(Theta>270 && Theta <=359){								// Top left quadrant
			xObject=odo.getX()-(Math.sin(Math.toRadians(90-(Theta-270)))*dist);
		}
		return xObject;
	}

	// Return Object's Y-coordinate
	public double targetY(double dist){
		double Theta = odo.getTheta();								// Angle of robot is facing:
		if(Theta>=0 && Theta<=90){										// Top right quadrant
			yObject=odo.getY()+(Math.sin(Math.toRadians(90-Theta))*dist);
		}else if(Theta>90 && Theta <=180){								// Bottom right quadrant
			yObject=odo.getY()-(Math.sin(Math.toRadians(Theta-90))*dist);
		}else if(Theta>180 && Theta <=270){								// Bottom left quadrant
			yObject=odo.getY()-(Math.sin(Math.toRadians(90-(Theta-180)))*dist);
		}else if(Theta>270 && Theta <=359){								// Top left quadrant
			yObject=odo.getY()+(Math.sin(Math.toRadians(Theta-270))*dist);
		}
		return yObject;
	}

	// Check if out of bound
	public boolean OoB(double x, double y){
		if(x<-15 || x>75 || y<-15 || y>75){
			return true;
		}else
			return false;
	}


	//Coin sound!
	public void coinSound(){
		//Sound.playNote(a, 988, 100);
		Sound.playNote(a, 1319, 300);
	}

	//Backing up method
	public void backUp(){
		while(distance < 12){
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(100);
			leftMotor.backward();
			rightMotor.backward();
		}
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
	}

	//travelTo sets (x,y) and angle of destination 
	public void travelTo(double x, double y){
		xDest = x;
		yDest = y;
		yCurrent = odo.getY();
		xCurrent = odo.getX();
		thetaDest = Math.toDegrees(Math.atan2((xDest-xCurrent), (yDest-yCurrent)));
		isNavigating = true;
	}

	//determine to which angle the robot has to turn to
	public void turnTo(double theta){
		//
		isNavigating = true;
		double deltaTheta = ((theta) - (odo.getTheta()));
		if(Math.abs(deltaTheta) < 180){
			//deltaTheta
			turn(deltaTheta);
		}else if(deltaTheta < -180){
			deltaTheta += 360;
			turn(deltaTheta);
			//turns right
		}	
		else if(deltaTheta > 180){
			deltaTheta -= 360;
			//turns left
			turn(deltaTheta);
		}
		thetaCurrent = odo.getTheta();
	}	

	//Turning method
	public void turn(double theta){
		leftMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta), false);
	}

	//Move forward method
	public void forward(){
		isNavigating = true;
		xCurrent = odo.getX();
		yCurrent = odo.getY();
		leftMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);
		leftMotor.setSpeed(FORWARD_SPEED+1);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();
	}

	//Method for motors to stop at the same time
	public void mStop(){
		isNavigating = false;
		leftMotor.stop(true);
		rightMotor.stop(true);
	}

	//convert distance to the angle of rotation of the wheels in degrees
	public int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	//convert cart angle to the angle of rotation of the wheels in degrees
	public int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	//Wrap Angle method
	public double wrapAngle(double angle){
		if(angle<0){
			angle+=360;
		}else if(angle>360){
			angle-=360;
		}
		return angle;
	}

	//Updates the destination angle from current position
	public double getDestAngle(){
		yCurrent = odo.getY();
		xCurrent = odo.getX();
		thetaDest = Math.toDegrees(Math.atan2((xDest-xCurrent), (yDest-yCurrent)));
		return thetaDest;
	}



	// Boolean Methods
	// Checking methods

	//Check if robot is navigating
	public boolean isNavigating(){
		return isNavigating;	
	}

	public void setNavigating(boolean a){
		isNavigating = a;
	}

	//Check if robot is facing angle of destination
	public boolean facingDest(){
		thetaCurrent = (odo.getTheta());
		//System.out.println("       BBB:"+(int)wrapAngle(thetaDest));
		if(Math.abs(thetaCurrent-wrapAngle(thetaDest))<2){
			return true;
		}else 
			return false;
	}

	public double getAngleDiffCW(double start, double end){
		if(start > end){
			return 360 - start + end;
		} else {
			return end - start;
		}
	}

	//Setter method for corner
	public void setCorner(int[] a){
		this.corner = a;
	}

	//Set zone
	public void setGoodZone(int[] b){
		this.goodZone = b;
	}

	public void scanCorrect(){
		nav.turnTo(wrapAngle(odo.getTheta()-45));
		boolean needCorrect=false;
		while(usLower.filterData()<10){
			needCorrect=true;
			nav.rotateOnSpot(SCAN_SPEED);
		}
		if(needCorrect){
			nav.turnTo(wrapAngle(odo.getTheta()+15));
		}
		return;
	}

	// Inherited methods from UScontroller
	@Override
	public void processUSData(double distance) {
		this.distance = distance;
		if(!notScanning){ 
			if(this.distance < rangeObject && this.distance > 3){
				distObject = this.distance;
				LCD.drawString("OBJECT DETECTED", 0, 6);
				isDetected = true;
			}else{
				LCD.drawString("              ", 0, 7);
				isChecked=false;
				LCD.drawString("Nothing         ", 0, 6);
				isDetected = false;
			}
		}
	}

	@Override
	public double readUSDistance() {
		return this.distance;
	}
	private static void Victory(int[] a){
		Sound.playNote(a, 440, 110);
		Sound.playNote(a, 587, 110);
		Sound.playNote(a, 740, 110);
		Sound.playNote(a, 880, 220);
		Sound.playNote(a, 740, 110);
		Sound.playNote(a, 880, 320);
	}

}
