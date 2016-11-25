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
	private static final int FORWARD_SPEED = 150, ROTATE_SPEED = 100, ACCELERATION = 1000, SCAN_SPEED = 20, SCAN_SIZE = 25, ID_SIZE = 35, ID_DISTANCE = 6, IGNORE_TIME = 1000;
	private static final double SCAN_RADIUS = 60.96, ID_RADIUS = 15;
	private static final double SAFE_DETECTION = 40;
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
	private boolean localized = false;
	private USPoller usLower;
	private USPoller usUpper;
	private double scanStartAngle;
	private int[] corner = new int[2];
	private int[] zone = new int[4];


	// Enum declaration/initialization
	enum State {INIT, SCAN, TURNING, TRAVELLING, IDENTIFY, EMERGENCY, TARGET};


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


	}

	// Setup Threads
	AvoidObstacle avoidance = null;

	// Run method
	/**
	 * Method that extends from Thread
	 * This method contains the main State Machine
	 * that handles the search for styrofoam blocks
	 */
	public void run(){
		State state = State.INIT;
		while(true){
			switch(state){
			case INIT:
				LCD.drawString("            ", 0, 5);
				LCD.drawString("INIT", 0, 5);
				double angle;
				if(!localized){
					odo.getPosition(new double[]{0, 0, 0}, new boolean[]{true, true, true});
					nav.getOdometerInfo();
					localization.localize();
					localized = true;
					if(corner[0] == 0){
						if(corner[1] == 0){
							angle = 0;
						} else {
							angle = 90;
						}
					}else{
						if(corner[1] == 0){
							angle = 270;
						} else {
							angle = 180;
						}
					}
					odo.setPosition(new double[]{corner[0],corner[1], angle}, new boolean[]{true, true, true});
				}
				nav.getOdometerInfo();
				Victory(a);
				LCD.drawString("Corner x: " + Integer.toString(corner[0]), 0, 6);
				LCD.drawString("Corner y: " + Integer.toString(corner[1]), 0, 7);
				state = State.SCAN;
				break;

			case SCAN:
				LCD.drawString("            ", 0, 5);
				LCD.drawString("SCAN", 0, 5);
				usLower.enable();
				scanStartAngle = (odo.getTheta())*(180.0/Math.PI);
				ScanQueue scanQueue = new ScanQueue(SCAN_SIZE, SCAN_RADIUS);
				ScanQueue idQueue = new ScanQueue(ID_SIZE, ID_RADIUS);
				ScanQueue closeQueue = new ScanQueue(3, 100);
				double distance, startTime, currentTime;
				boolean block = true;
				boolean moved;
				double[] scanLocation = {corner[0], corner[0], 0};
				double[] idPosition = {0, 0, 0};
				odo.getPosition(scanLocation, new boolean[] {true, true, true});
				while(odo.getTheta()*(180.0/Math.PI) <= scanStartAngle + 90){
					nav.rotateOnSpot(SCAN_SPEED);
					distance = usLower.filterData();
					moved = false;
					if(block){
						if(scanQueue.checkAndAddUnder(distance)){
							nav.stopMotors();
							coinSound();
							usLower.disable();
							double average = scanQueue.getAverage();
							if(average < SAFE_DETECTION){
								nav.turnTo(30);
							}
							double closingDistance = average - ID_DISTANCE;
							if(closingDistance > 0){
								nav.driveDistanceForward(closingDistance);
								moved = true;
							}
							scanQueue.clearQueue();
							//Pull and update values for ID_TIME. Update block boolean if we detect it
							usUpper.enable();

							startTime = System.currentTimeMillis();
							currentTime = System.currentTimeMillis();
							while(currentTime - startTime < IGNORE_TIME){
								distance = usUpper.filterData();
								if(idQueue.checkAndAddUnder(distance)){
									block = false;
									Sound.beep();
								}
								currentTime = System.currentTimeMillis();
							}
							usUpper.disable();
							//TODO write code for picking up block
							if(block){
								Sound.playNote(a, 440, 250);
								moved = true;
								blockCatch();
								nav.travelTo(zone[0], zone[1]);
								nav.turnTo(nav.getAngle(zone[0], zone[1], zone[2], zone[3], nav.getOrientation()));
								//lift1.rotate(-2450, true);
								//lift2.rotate(-2450, false);
								claw.rotateTo(0);
								break;
								//claw.flt();

							} else {
								Sound.beep();

							}
							if(moved){
								odo.getPosition(idPosition, new boolean[]{true, true, true});
								nav.getOdometerInfo();
								nav.travelTo(scanLocation[0], scanLocation[1]);
								nav.turnTo(nav.getAngle(odo.getX(), odo.getY(), idPosition[0], idPosition[1], nav.getOrientation()));
								if(block){
									System.exit(0);
								}
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
				break;

			case TURNING:								    //TURNING STATE
				LCD.drawString("            ", 0, 5);
				LCD.drawString("TURNING", 0, 5);
				turnTo(getDestAngle());					//Updates Destination Angle and turn to that Angle
				if(facingDest()){						//check if facing destination
					state = State.TRAVELLING;
				}
				break;	
			case TRAVELLING:								//TRAVELLING STATE
				forward();
				LCD.drawString("            ", 0, 5);
				LCD.drawString("TRAVELLING", 0, 5);

				xCurrent = odo.getX();
				yCurrent = odo.getY();
				if(((Math.abs(xCurrent-xDest)<1.3) && (Math.abs(yCurrent-yDest)<1.3))){ 										//Arrived to destination
					mStop();								//Stop Driving
					state = State.INIT;							
				}
				break;
			case IDENTIFY:									// IDENTIFY STATE
				
				if(isChecked && isBlock){

					state = State.TARGET;
				}else if(isChecked && !isBlock){					//Check if there is an obstacle
					notScanning=false;
					state = State.EMERGENCY;
					backUp();
					// Create new AvoidObstacle thread every time
					avoidance = new AvoidObstacle(this);
					usLower.disable();
					usUpper.enable();
					avoidance.start();
				}
				break;
			case EMERGENCY:									//EMERGENCY STATE
				LCD.drawString("            ", 0, 5);
				LCD.drawString("EMERGENCY", 0, 5);
				break;
			case TARGET:								//Catch block and travel to goal
				LCD.drawString("            ", 0, 5);
				LCD.drawString("TARGET", 0, 5);

				//travelTo goal
				
				nav.travelTo(corner[0], corner[1]);
				state = State.INIT;

			}
			try{											//Thread to sleep for 30miliseconds
				Thread.sleep(30);
			}catch(InterruptedException e){					//catch exception if current thread is interrupted
				e.printStackTrace();
			}
		}
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
		liftDown();
		int turnAngle=20;
		int distForward=20;
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, turnAngle), true);
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, turnAngle), false);
		leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, turnAngle), true);
		rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, turnAngle), false);
		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, distForward), true);
		rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, distForward), false);
		grab();
		liftUp();
	}
	
	
	public void liftDown() {
		lift1.setAcceleration(500);
		lift2.setAcceleration(500);
		lift1.setSpeed(100);
		lift2.setSpeed(100);
//		lift1.rotate(-1470,true);
//		lift2.rotate(-1470,false);
		lift1.rotate(-1070,true);
		lift2.rotate(-1070,false);
		claw.setAcceleration(80);
		claw.setSpeed(50);
		claw.rotateTo(40);
		lift1.rotate(-400,true);
		lift2.rotate(-400,false);
	}
	
	public void liftUp(){
		lift1.setAcceleration(500);
		lift2.setAcceleration(500);
		lift1.setSpeed(100);
		lift2.setSpeed(100);
		lift1.rotate(1470,true);
		lift2.rotate(1470,false);
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


	//Setter method for corner
	public void setCorner(int[] a){
		this.corner = a;
	}

	//Set zone
	public void setZone(int[] b){
		this.zone = b;
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
