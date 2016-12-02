package master;

import lejos.hardware.Sound;
import master.poller.USPoller;

/**
 *	ScanThred runs the US Poller and sets Alert boolean the Navigation to true 
 *	when it senses an object at a certain range. 
 *	Extends thread to run simultaneously with other operations.
 * 
 * @author Jerome Marfleet
 * @author Yu-Yueh Liu
 * @since 2016-11-29
 *
 */
public class ScanThread extends Thread{
	private Navigation nav;
	private USPoller us;
	private ScanQueue queue;
	private boolean over;
	private boolean end;
	
	/**
	 * Constructor 
	 * 
	 * @param nav Navigation Class
	 * @param us US Poller
	 * @param size size of ScanQueue
	 * @param boundary Boundary for values in ScanQueue
	 * @param above True to scan data greater than Boundary, False otherwise
	 */
	public ScanThread(Navigation nav, USPoller us, int size, double boundary, boolean above){
		this.nav = nav;
		this.us = us;
		queue = new ScanQueue(size, boundary);
		this.over = above;
		
	}
	
	/**
	 * Method extended from Thread
	 */
	public void run(){
		while(!end){
			if(over){
				if(queue.checkAndAddOver(us.filterData())){
					nav.setAlert(true);
					Sound.beep();
				}
			} else {
				if(queue.checkAndAddUnder(us.filterData())){
					nav.setAlert(true);
					Sound.beep();
				}
			}
		}
	}
	
	/**
	 * Method to Stop the thread
	 */
	public void end(){
		end = true;
	}
	
}
