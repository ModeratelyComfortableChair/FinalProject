/* DPM Team 12
 * 
 */
package master;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import master.localization.LocalizationMaster;
import master.odometry.Odometer;
import master.poller.USPoller;

/**
 * Search will perform Scanning once the robot finishes Localizing to (0,0)
 * Once a block is captured, it will move to the right zone to drop the block
 * and come back to the starting corner.
 * 
 * @author Yu-Yueh Liu
 * @author Jerome Marfleet
 * @version 2.0
 * @since 2016-11-28
 *
 */
public class Search extends Thread{
	// Constants for navigation 
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
	 * @param lift1 Pulley Motor 1
	 * @param lift2 Pulley Motor 2
	 * @param claw Claw motor
	 * @param localization Localization Class
	 * @param usLower Lower US Sensor
	 * @param usHigher Upper Us Sensor
	 */
	public Search(Odometer odometer, Navigation nav, EV3LargeRegulatedMotor lift1, EV3LargeRegulatedMotor lift2,
			RegulatedMotor claw, LocalizationMaster localization, USPoller usLower, USPoller usHigher) {
		//Set variables
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
						currentTile = new Tile(1, 10);
					}
				}else{
					if(corner[1] == 0){
						angle = 270;
						cornerNum = 2;
						currentTile = new Tile(10, 1);
					} else {
						angle = 180;
						cornerNum = 3;
						currentTile = new Tile(10, 10);
					}
				}
				scanEnd = currentTile;
				odo.setPosition(new double[]{corner[0],corner[1], angle*(Math.PI/180.0)}, new boolean[]{true, true, true});

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
								nav.setForwardSpeed(250);
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
								nav.setForwardSpeed(250);
								if(blockCount == 1){
									state=State.TRAVELLING;
									scanEnd = new Tile(findTile(odo.getX()), findTile(odo.getY()));
									nav.travelTo(scanEnd.getCenterX(), scanEnd.getCenterY(), false);
									break;
								}
							} else {
								Sound.beep();
								nav.driveDistanceBackward(10, false);
							}
							if(moved){
								odo.getPosition(idPosition, new boolean[]{true, true, true});
								nav.getOdometerInfo();
								nav.setForwardSpeed(250);
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
				Tile collection = new Tile(findTile(goodZone[0] - 30.48), findTile(goodZone[1] - 30.48));
				nav.getOdometerInfo();
				Tile endZone = avoider.findPath(collection.getX(), collection.getY(), scanEnd.getX(), scanEnd.getY());
				//If the tile we wish to drop off at is clear.
				if(collection.getX() != endZone.getX() || collection.getX() != collection.getY()){
					collection = new Tile(findTile(goodZone[0]), findTile(goodZone[0]-30.48));
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
		return (int)(coord/30.48 + 1);
	}

	public void blockCatch(){
		liftDownToGrab();
		int turnAngle=20;
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

	//Coin sound!
	public void coinSound(){
		//Sound.playNote(a, 988, 100);
		Sound.playNote(a, 1319, 300);
	}
	
	//convert distance to the angle of rotation of the wheels in degrees
	public int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	//convert cart angle to the angle of rotation of the wheels in degrees
	public int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
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
		nav.rotate(-20);
		boolean needCorrect=false;
		while(usLower.filterData()>12){
			needCorrect=true;
			nav.rotateOnSpot(SCAN_SPEED);
		}
		if(needCorrect){
			nav.rotate(10);
		}
		return;
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
