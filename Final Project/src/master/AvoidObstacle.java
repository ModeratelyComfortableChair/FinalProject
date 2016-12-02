/* DPM Team 12
 * 
 */
package master;

import java.util.Stack;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import master.odometry.Odometer;
import master.poller.USPoller;

/**
 * AvoidObstacle will avoid the Obstacle once the thread is started
 * until the robot is safe.
 * 
 * @author Yu-Yueh Liu
 * @version 1.0
 * @since 2016-11-07
 *
 */

public class AvoidObstacle extends Thread {

	private final double TILE_WIDTH = 30.48;
	private final double DETECT_RANGE = 4;
	private final int DETECT_SIZE = 7;
	private Navigation nav;
	private USPoller us;
	private ScanThread scan;
	private Odometer odo;
	private String lastAvoidance = "none";
	//constructor
	/**
	 * Constructor 
	 * 
	 * @param nav Navigation Class
	 * @param us US Poller Class
	 * @param odo Odometer Class
	 */
	public AvoidObstacle(Navigation nav, USPoller us, Odometer odo) {
		this.nav = nav;
		this.us = us;
		this.odo = odo;
	}

	//obstacle avoidance - get around the blocks
	/**
	 * Method that extends from Thread
	 * This method will run continuously until the robot is safe
	 */
	public Tile findPath(int destX, int destY, int currentX, int currentY){
		ScanThread detector = null;
		int xCount, yCount;
		Tile nextTile;
		double[] nextCenter;
		nav.getOdometerInfo();
		Stack<Tile> path = new Stack<>();
		path.push(new Tile(currentX, currentY));
		while (destX != currentX || destY != currentY){
			detector = new ScanThread(nav, us, DETECT_SIZE, DETECT_RANGE, false);
			us.enable();
			detector.start();
			int count = 0;
			xCount = destX - currentX;
			yCount = destY - currentY;
			if(xCount > 0){ //there x is greater than our x
				nextTile = new Tile(currentX + 1, currentY);
				nextCenter = nextTile.getCenter();
				nav.travelTo(nextCenter[0], nextCenter[1], true);
				if(nav.getAlert()){
					detector.end();
					nav.driveDistanceBackward(10, false);
					nav.getOdometerInfo();
					us.disable();
					nav.travelTo(path.peek().getCenterX(), path.peek().getCenterY(), false);
					if(xCount == 1 && yCount == 0){
						return path.peek();
					}
					if(yCount >= 0 && lastAvoidance != "negY"){//turn right first and try. If you fail, turn left
						nav.turnTo(90);
						nav.setAlert(false);
						count = getAroundTile(-90, path.peek());
						if(count != 0){
							for(int i = 1; i <= count; i++){
								currentY++;
								path.push(new Tile(currentX, currentY));
							}
							lastAvoidance = "posY";
							currentX++;
							path.push(new Tile(currentX, currentY));
						} else {
							nav.setAlert(false);
							count = getAroundTile(90, path.peek());
							if(count != 0){
								for(int i = 1; i <= count; i++){
									currentY--;
									path.push(new Tile(currentX, currentY));
								}
								lastAvoidance = "negY";
								currentX++;
								path.push(new Tile(currentX, currentY));
							}
						}
					} else {//turn left first. IT it fails, try right
						nav.turnTo(-90);
						nav.setAlert(false);
						count = getAroundTile(90, path.peek());
						if(count != 0){
							for(int i = 1; i <= count; i++){
								currentY--;
								path.push(new Tile(currentX, currentY));
							}
							lastAvoidance = "negY";
							currentX++;
							path.push(new Tile(currentX, currentY));
						} else {
							nav.setAlert(false);
							count = getAroundTile(-90, path.peek());
							if(count != 0){
								for(int i = 1; i <= count; i++){
									currentY++;
									path.push(new Tile(currentX, currentY));
								}
								lastAvoidance = "posY";
								currentX++;
								path.push(new Tile(currentX, currentY));
							}
						}
					}
				} else {
					path.push(nextTile);
					currentX++;
				}
			} else if (xCount < 0){
				nextTile = new Tile(currentX - 1, currentY);
				nextCenter = nextTile.getCenter();
				nav.travelTo(nextCenter[0], nextCenter[1], true);
				if(nav.getAlert()){
					detector.end();
					nav.driveDistanceBackward(10, false);
					nav.getOdometerInfo();
					us.disable();
					nav.travelTo(path.peek().getCenterX(), path.peek().getCenterY(), false);
					if(xCount == -1 && yCount == 0){
						return path.peek();
					}
					if(yCount >= 0 &&lastAvoidance != "negY"){//turn right first and try. If you fail, turn left
						nav.turnTo(-90);
						nav.setAlert(false);
						count = getAroundTile(90, path.peek());
						if(count != 0){
							for(int i = 1; i <= count; i++){
								currentY++;
								path.push(new Tile(currentX, currentY));
							}
							lastAvoidance = "posY";
							currentX--;
							path.push(new Tile(currentX, currentY));
						} else {
							nav.setAlert(false);
							count = getAroundTile(-90, path.peek());
							if(count != 0){
								for(int i = 1; i <= count; i++){
									currentY--;
									path.push(new Tile(currentX, currentY));
								}
								lastAvoidance = "negY";
								currentX--;
								path.push(new Tile(currentX, currentY));
							}
						}
					} else {//turn left first. IT it fails, try right
						nav.turnTo(90);
						nav.setAlert(false);
						count = getAroundTile(-90, path.peek());
						if(count != 0){
							for(int i = 1; i <= count; i++){
								currentY--;
								path.push(new Tile(currentX, currentY));
							}
							lastAvoidance = "negY";
							currentX--;
							path.push(new Tile(currentX, currentY));
						} else {
							nav.setAlert(false);
							count = getAroundTile(90, path.peek());
							if(count != 0){
								for(int i = 1; i <= count; i++){
									currentY++;
									path.push(new Tile(currentX, currentY));
								}
								lastAvoidance = "posY";
								currentX--;
								path.push(new Tile(currentX, currentY));
							}
						}
					}
				} else {
					path.push(nextTile);
					currentX--;
				}
			} else if (yCount > 0){
				nextTile = new Tile(currentX, currentY+1);
				nextCenter = nextTile.getCenter();
				nav.travelTo(nextCenter[0], nextCenter[1], true);
				if(nav.getAlert()){
					detector.end();
					nav.driveDistanceBackward(10, false);
					nav.getOdometerInfo();
					us.disable();
					nav.travelTo(path.peek().getCenterX(), path.peek().getCenterY(), false);
					if(xCount == 0 && yCount == 1){
						return path.peek();
					}
					if(xCount >= 0 && lastAvoidance != "negX"){//turn right first and try. If you fail, turn left
						nav.turnTo(-90);
						nav.setAlert(false);
						count = getAroundTile(90, path.peek());
						if(count != 0){
							for(int i = 1; i <= count; i++){
								currentX++;
								path.push(new Tile(currentX, currentY));
							}
							lastAvoidance = "posX";
							currentY++;
							path.push(new Tile(currentX, currentY));
						} else {
							nav.setAlert(false);
							count = getAroundTile(-90, path.peek());
							if(count != 0){
								for(int i = 1; i <= count; i++){
									currentX--;
									path.push(new Tile(currentX, currentY));
								}
								lastAvoidance = "negX";
								currentY++;
								path.push(new Tile(currentX, currentY));
							}
						}
					} else {//turn left first. IT it fails, try right
						nav.turnTo(90);
						nav.setAlert(false);
						count = getAroundTile(-90, path.peek());
						if(count != 0){
							for(int i = 1; i <= count; i++){
								currentX--;
								path.push(new Tile(currentX, currentY));
							}
							lastAvoidance = "negX";
							currentY++;
							path.push(new Tile(currentX, currentY));
						} else {
							nav.setAlert(false);
							count = getAroundTile(90, path.peek());
							if(count != 0){
								for(int i = 1; i <= count; i++){
									currentX++;
									path.push(new Tile(currentX, currentY));
								}
								lastAvoidance = "posX";
								currentY++;
								path.push(new Tile(currentX, currentY));
							}
						}
					}
				} else {
					path.push(nextTile);
					currentY++;
				}
			} else if (yCount < 0){
				nextTile = new Tile(currentX, currentY-1);
				nextCenter = nextTile.getCenter();
				nav.travelTo(nextCenter[0], nextCenter[1], true);
				if(nav.getAlert()){
					detector.end();
					nav.driveDistanceBackward(10, false);
					nav.getOdometerInfo();
					us.disable();
					nav.travelTo(path.peek().getCenterX(), path.peek().getCenterY(), false);
					if(xCount == 0 && yCount == -1){
						return path.peek();
					}
					if(xCount >= 0 && lastAvoidance != "negX"){//turn right first and try. If you fail, turn left
						nav.turnTo(90);
						nav.setAlert(false);
						count = getAroundTile(-90, path.peek());
						if(count != 0){
							for(int i = 1; i <= count; i++){
								currentX++;
								path.push(new Tile(currentX, currentY));
							}
							lastAvoidance = "posX";
							currentY--;
							path.push(new Tile(currentX, currentY));
						} else {
							nav.setAlert(false);
							count = getAroundTile(90, path.peek());
							if(count != 0){
								for(int i = 1; i <= count; i++){
									currentX--;
									path.push(new Tile(currentX, currentY));
								}
								lastAvoidance = "negX";
								currentY--;
								path.push(new Tile(currentX, currentY));
							}
						}
					} else {//turn left first. IT it fails, try right
						nav.turnTo(-90);
						nav.setAlert(false);
						count = getAroundTile(90, path.peek());
						if(count != 0){
							for(int i = 1; i <= count; i++){
								currentX--;
								path.push(new Tile(currentX, currentY));
							}
							lastAvoidance = "negX";
							currentY--;
							path.push(new Tile(currentX, currentY));
						} else {
							nav.setAlert(false);
							count = getAroundTile(-90, path.peek());
							if(count != 0){
								for(int i = 1; i <= count; i++){
									currentX++;
									path.push(new Tile(currentX, currentY));
								}
								lastAvoidance = "posX";
								currentY--;
								path.push(new Tile(currentX, currentY));
							}
						}
					}
				} else {
					path.push(nextTile);
					currentY--;
				}
			}
		}
		nav.getOdometerInfo();
		detector.end();
		nav.setAlert(false);
		us.disable();
		return new Tile(destX, destY);
	}

	/**
	 * 
	 * @param angle angle between desired position and avoidance position.
	 * @param startTile initial tile we start at
	 * @return count representing number of tiles we moved to avoid. 0 if we didn't avoid
	 */
	public int getAroundTile(int angle, Tile startTile){
		us.enable();
		nav.getOdometerInfo();
		int count = 0;
		scan = new ScanThread(nav, us, DETECT_SIZE, DETECT_RANGE, false);
		double position[] = {0,0,0};
		scan.start();
		while(true){
			nav.driveDistanceForward(TILE_WIDTH, true);	//Drive across
			if(!nav.getAlert()){						//Drove uninterrupted			
				nav.turnTo(-angle);						//Turn back to desired direction
				count++;
				nav.getOdometerInfo();
				odo.getPosition(position, new boolean[]{true, true, true});
				nav.driveDistanceForward(TILE_WIDTH, true);	//Drive forward to desired direction
				if(!nav.getAlert()){					//We got to the tile with no interruptions. break
					break;
				} else {								//Tile not clear. go back to new tile and try again
					nav.driveDistanceBackward(10, false);
					nav.getOdometerInfo();
					nav.travelTo(position[0], position[1], false);
					nav.turnTo(-angle);
					nav.setAlert(false);
				}
			} else{										//Cannot go in avoidance direction
				scan.end();
				nav.driveDistanceBackward(10, false);
				nav.getOdometerInfo();
				nav.setAlert(false);
				nav.travelTo(startTile.getCenterX(), startTile.getCenterY(), false); //travel to start tile
				count = 0;
				break;
			}


		}
		nav.getOdometerInfo();
		scan.end();
		us.disable();
		return count;
	}

}
