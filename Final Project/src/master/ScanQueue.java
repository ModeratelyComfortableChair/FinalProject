package master;

/**
 * ScanQueue takes values, mainly from a USPoller, and places them in a queue. It can also return whether or not those values are all under a certain value.
 * @author Jerome Marfleet
 * @version 1.0
 * @since 2016-11-13
 */
public class ScanQueue {
	private int queueSize, count;
	private double boundary;
	private double[] queue;
	
	/**
	 * Constructor
	 * @param stackSize size of the stack
	 * @param boundary cutoff value
	 */
	public ScanQueue(int stackSize, double boundary){
		this.queueSize = stackSize;
		this.boundary = boundary;
		queue = new double[queueSize];
		count = 0;
	}
	
	/**
	 * Checks to see if
	 * @return
	 */
	public boolean isFull(){
		if(count == queueSize){
			return true;
		}
		return false;
	}
	
	/**
	 * Obtains the average of the quee
	 * @return the average of every value in the queue
	 */
	public double getAverage(){
		double sum = 0;
		for (int i  = 0; i < queueSize; i++){
			sum += queue[i];
		}
		return sum/count;
	}
	
	/**
	 * 
	 * @param distance
	 * @return adds a new value to the queue, and returns true if the queue is filled and every value is below the boundary
	 *
	 */
	public boolean checkAndAddUnder(double distance){
		if(distance == 0){
			return false;
		}
		if(count < queueSize){
			queue[count] = distance;
			count++;
			return false;
		} else {
			shiftQueue();
			queue[queueSize - 1] = distance;
			return checkQueueUnder();
		}
	}
	public boolean checkAndAddOver(double distance){
		if(distance == 0){
			return false;
		}
		if(count < queueSize){
			queue[count] = distance;
			count++;
			return false;
		} else {
			shiftQueue();
			queue[queueSize - 1] = distance;
			return checkQueueOver();
		}
	}
	
	/**
	 * Shifts every value in the queue one over.
	 */
	private void shiftQueue() {
		for(int i = 0; i < queueSize - 1; i++){
			queue[i] = queue[i + 1];
		}
		
	}

	/**
	 * Checks the queue to see if it is filled, and every value is below the boundary
	 * @return true if queue is filled and every value is below boundary. false otherwise.
	 */
	public boolean checkQueueUnder(){
		if(count < queueSize){
			return false;
		} 
		for(int i = 0; i < queueSize; i++){
			if(queue[i] > boundary){
				return false;
			}
		}
		
		return true;
	}
	public boolean checkQueueOver(){
		if(count < queueSize){
			return false;
		} 
		for(int i = 0; i < queueSize; i++){
			if(queue[i] < boundary){
				return false;
			}
		}
		
		return true;
	}
	
	/**
	 * Sets the count to 0. This is more efficient than filling stack with zeroes
	 */
	public void clearQueue(){
		count = 0;
	}
}
