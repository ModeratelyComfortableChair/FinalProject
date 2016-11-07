package finalproject.localization;

/**
 * LocalizationMaster handles the localization of the robot. Localization controls the order of
 * execution for the different implementations of the Localizer interface.
 * 
 * @author Jerome Marfleet
 * @author Yu-Yueh Liu
 * @version 1.0
 * @since 2016-11-06
 *
 */
public class LocalizationMaster {

	private Localizer usLocalizer, lightLocalizer;
	
	/**
	 * 
	 * @param usLocalizer
	 * @param lightLocalizer
	 */
	public LocalizationMaster(USLocalizer usLocalizer, LightLocalizer lightLocalizer){
		this.usLocalizer = usLocalizer;
		this.lightLocalizer = lightLocalizer;
	}
	
	/**
	 * Method calls localize on each individually Localizer instances.
	 */
	public void localize(){
		usLocalizer.localize();
		lightLocalizer.localize();
	}
}
