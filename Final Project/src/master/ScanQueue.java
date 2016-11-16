package master;

public class ScanQueue {
	private int queueSize, count;
	private double boundary;
	private double[] queue;
	
	
	public ScanQueue(int stackSize, double boundary){
		this.queueSize = stackSize;
		this.boundary = boundary;
		queue = new double[queueSize];
		count = 0;
	}
	
	public boolean isFull(){
		if(count == queueSize){
			return true;
		}
		return false;
	}
	
	public double getAverage(){
		double sum = 0;
		for (int i  = 0; i < queueSize; i++){
			sum += queue[i];
		}
		return sum/queueSize;
	}
	
	public boolean checkAndAdd(double distance){
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
			return checkQueue();
		}
	}
	
	private void shiftQueue() {
		for(int i = 0; i < queueSize - 1; i++){
			queue[i] = queue[i + 1];
		}
		
	}

	public boolean checkQueue(){
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
	
	public void clearQueue(){
		count = 0;
	}
}
