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
	private static final Port usUpperPort = null;
	private static final Port usLowerPort = LocalEV3.get().getPort("S3");
	private static final Port colorBackPort = LocalEV3.get().getPort("S4");	
	private static final Port colorLeftPort = null;
	private static final Port colorRightPort = null;
	
	public static final double WHEEL_RADIUS = 2.134;
	public static final double TRACK = 16.005; 
	
	private static Odometer odo;

	
	public static void main(String[] args) throws InterruptedException {
		int buttonChoice;
		int [] a = {4, 25, 500, 7000, 5};	// Array that determines instrument: Piano
		
		//Set up Escape thread
		(new Thread(){
			public void run(){
				Escape.testForEscape();
			}
		}).start();
		
		//Setup ultrasonic sensor
		// 1. Create a port object attached to a physical port (done above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating mode
		// 4. Create a buffer for the sensor data
		@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usLowerPort);		// usSensor is the instance
		SampleProvider usValue = usSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[usValue.sampleSize()];		// usData is the buffer in which data are returned

		//Setup color sensor
		// 1. Create a port object attached to a physical port (done above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating mode
		// 4. Create a buffer for the sensor data
		@SuppressWarnings("resource")
		SensorModes colorSensor = new EV3ColorSensor(colorBackPort);	// colorSensor is the instance
		SampleProvider colorValue = colorSensor.getMode("RGB");		// colorValue provides samples from this instance
		float[] colorData = new float[colorValue.sampleSize()];		// colorData is the buffer in which data are returned
						
		
		// some objects that need to be instantiated
		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RADIUS);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odo,t);
		Navigation nav = new Navigation(odo, leftMotor, rightMotor);
		
		USPoller usPoller = new USPoller(usValue, usData);
		LightPoller lightPoller = new LightPoller(colorValue, colorData);
		
		//Localization and it's localizer arguments
		USLocalizer usLocalizer = new USLocalizer(odo, usPoller, nav);
		LightLocalizer lightLocalizer = new LightLocalizer(odo, lightPoller, nav);
		LocalizationMaster localization = new LocalizationMaster(usLocalizer, lightLocalizer);
		
		
		Search searcher = new Search(odo, nav, turner, hook, colorValue, colorData, localization);
		
		
		
		
		
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

		if (buttonChoice == Button.ID_LEFT) {
			
			
		}else{
			
			// start the odometer, the odometry display
			odo.start();
			odometryDisplay.start();
			usPoller.start();
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