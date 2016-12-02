package master;

import master.odometry.Odometer;
/**
 * Tile Object
 * 
 * @author Jerome Marfleet
 * @since 2016-11-29
 */
public class Tile {

	private double center[];
	private final static double TILE_WIDTH = 30.48;
	private int x, y;
	
	/**
	 * Constructor 
	 * 
	 * @param x Bottom left X Coordinate of the Tile
	 * @param y Bottom left Y Coordinate of the Tile
	 */
	public Tile(int x, int y){
		this.x = x;
		this.y = y;
		center = new double[]{TILE_WIDTH*(x - 0.5), TILE_WIDTH*(y - 0.5)};
	}
	
	/**
	 * Get Center of the Tile
	 * @return the coordinate for the center of the Tile
	 */
	public double[] getCenter(){
		return center;
	}
	
	/**
	 * Get X coordinate of the center of the Tile
	 * @return the X coordinate for the center of the Tile
	 */
	public double getCenterX(){
		return center[0];
	}

	/**
	 * Get Y coordinate of the center of the Tile
	 * @return the Y coordinate for the center of the Tile
	 */
	public double getCenterY(){
		return center[1];
	}
	
	/**
	 * Get X value of Tile
	 * @return the bottom left X coordinate of Tile
	 */
	public int getX(){
		return x;
	}
	
	/**
	 * Get Y value of Tile
	 * @return the bottom left Y coordinate of Tile
	 */
	public int getY(){
		return y;
	}
	
	
}
