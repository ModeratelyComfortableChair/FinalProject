package master;

import master.odometry.Odometer;

public class Tile {

	private double center[];
	private final static double TILE_WIDTH = 30.48;
	private int x, y;
	public Tile(int x, int y){
		this.x = x;
		this.y = y;
		center = new double[]{TILE_WIDTH*(x - 0.5), TILE_WIDTH*(y - 0.5)};
	}
	public double[] getCenter(){
		return center;
	}
	public double getCenterX(){
		return center[0];
	}
	public double getCenterY(){
		return center[1];
	}
	public int getX(){
		return x;
	}
	public int getY(){
		return y;
	}
	
	
}
