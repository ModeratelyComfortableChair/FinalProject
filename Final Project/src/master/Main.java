/* DPM Team 12
 *
 */
package master;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import master.communication.Communication;
import master.localization.LightLocalizer;
import master.localization.LocalizationMaster;
import master.localization.Localizer;
import master.localization.USLocalizer;
import master.poller.LightPoller;
import master.poller.USPoller;


public class Main {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	//TODO Replace this with the comments below
	private static final EV3LargeRegulatedMotor hook = null;	// = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3MediumRegulatedMotor turner = null; //new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));	
	
	//TODO initialize other usPorts and colorPorts

	private static final Port colorBackPort = LocalEV3.get().getPort("S1");
	private static final Port usUpperPort = LocalEV3.get().getPort("S3");
	private static final Port usLowerPort = LocalEV3.get().getPort("S4");
	private static final Port colorLeftPort = null;
	private static final Port colorRightPort = null;
	
	//TODO Measure and obtain proper constants
	//increasing radius reduces distance and turning angle
	//increasing width increases turning
	
	//if I want to increase the travelling distance, i need to decrease track and radius by the same amount
	//If i want to increase the turning angle, I need to increase the track
	public static final double WHEEL_RADIUS = 2.134;
	public static final double TRACK = 17.5; 	
	public static Integer[] Data = new Integer[5];
	public static void main(String[] args) throws InterruptedException {
		int buttonChoice;
		int [] a = {4, 25, 500, 7000, 5};	// Array that determines instrument: Piano
		
		//Set up Escape thread
		(new Thread(){
			public void run(){
				Escape.testForEscape();
			}
		}).start();
		
		
		@SuppressWarnings("resource")							    			// Because we don't bother to close this resource
		EV3UltrasonicSensor usUpperSensor = new EV3UltrasonicSensor(usUpperPort);		// usSensor is the instance
		
		@SuppressWarnings("resource")							    			// Because we don't bother to close this resource
		SensorModes usLowerSensor = new EV3UltrasonicSensor(usLowerPort);		// usSensor is the instance
		
		@SuppressWarnings("resource")
		SensorModes lightBackSensor = new EV3ColorSensor(colorBackPort);
						
		
		// some objects that need to be instantiated
		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RADIUS);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odo,t);
		Navigation nav = new Navigation(odo, leftMotor, rightMotor);
		
		USPoller usLowerPoller = new USPoller(usUpperSensor);
		USPoller usUpperPoller = new USPoller(usLowerSensor);
		LightPoller lightBackPoller = new LightPoller(lightBackSensor);
		
		//Localization and it's localizer arguments
		USLocalizer usLocalizer = new USLocalizer(odo, usLowerPoller, nav);
		LightLocalizer lightLocalizer = new LightLocalizer(odo, lightBackPoller, nav);
		LocalizationMaster localization = new LocalizationMaster(usLocalizer, lightLocalizer);
		
		
		Search searcher = new Search(odo, nav, turner, hook, localization, usLowerPoller, usUpperPoller);
		Communication com = new Communication();
		
		
		
		
		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should navigate or to Navigate with obstacles
			t.drawString("< 	Center    >", 0, 0);
			t.drawString("_________________", 0, 1);
			t.drawString("      	       ", 0, 2);
			t.drawString("  	 GAME	   ", 0, 3);
			t.drawString("  	START! 	   ", 0, 4);
			t.drawString("_________________", 0, 5);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_ENTER) {
			//fetch Data
			Data=com.Communicate();
//			Data[0] is Corner
//			Data[1] is Zone Lower X
//			Data[2] is Zone Lower Y
//			Data[3] is Zone Upper X
//			Data[4] is Zone Upper Y
			
		}else{
			
			// start the odometer, the odometry display
			odo.start();
			odometryDisplay.start();
			usLowerPoller.start();
			searcher.start();
						
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
	
	// Victory sound!
	private static void Victory(int[] a){
		Sound.playNote(a, 440, 110);
		Sound.playNote(a, 587, 110);
		Sound.playNote(a, 740, 110);
		Sound.playNote(a, 880, 220);
		Sound.playNote(a, 740, 110);
		Sound.playNote(a, 880, 320);
	}
}
