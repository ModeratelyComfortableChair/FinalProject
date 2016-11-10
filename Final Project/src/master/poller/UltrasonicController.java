package master.poller;

public interface UltrasonicController {
	
	public void processUSData(double distance);
	
	public double readUSDistance();
}
