package master.localization;

/**
 * LocalizationMaster handles the localization of the robot. Localization controls the order of
 * execution for the different implementations of the Localizer interface.
 * 
 * @author Jerome Marfleet
 * @author Yu-Yueh Liu
 * @version 2.0
 * @since 2016-11-06
 *
 */
public class LocalizationMaster {

	private Localizer usLocalizer;
	
	/**
	 * Constructor
	 * 
	 * @param usLocalizer
	 */
	public LocalizationMaster(USLocalizer usLocalizer){
		this.usLocalizer = usLocalizer;
	}
	
	/**
	 * Method calls localize on each individually Localizer instances.
	 */
	//TODO un comment light localizer
	public void localize(){
		usLocalizer.localize();
		//lightLocalizer.localize();
	}
}
