package master;

import lejos.hardware.Sound;
import master.poller.USPoller;

public class ScanThread extends Thread{
	private Navigation nav;
	private USPoller us;
	private ScanQueue queue;
	private boolean over;
	private boolean end;
	
	
	public ScanThread(Navigation nav, USPoller us, int size, double boundary, boolean above){
		this.nav = nav;
		this.us = us;
		queue = new ScanQueue(size, boundary);
		this.over = above;
		
	}
	
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
	public void end(){
		end = true;
	}
	
}
