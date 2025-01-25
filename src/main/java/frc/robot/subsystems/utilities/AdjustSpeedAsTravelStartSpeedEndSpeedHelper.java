package frc.robot.subsystems.utilities;

//import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * 
 * @author richard.topolewski 
 * 
 */
public class AdjustSpeedAsTravelStartSpeedEndSpeedHelper extends AdjustSpeedAsTravelHelper{
	
	boolean debug = true;

    double m_startDistance          = 0.0d; // start position, in distance units for example inches or Degrees of rotation
    double m_startSpeed             = 0.0d; // start speed, in distance units for example inches or Degrees of rotation
    
	double m_endDistance            = 0.0d; // end position, in distance units for example inches or Degrees of rotation
    double m_endSpeed               = 0.0d; // end speed, in distance units for example inches or Degrees of rotation
    
    double m_ramp                   = 0.0d; // the ramp up/down in speed as we travel along from start to end distance
    	
    double m_currentMeasuredDistance  = 0.0d;
    double m_initialMeasuredDistance  = 0.0d;
 
  

	   	
	/**
     * This helper class just takes a start distance and speed, and and end distance and speed 
     *     and will provide a ramping of speed (either up or down)  to get to that end distance and be traveling at the end speed
     * @param aStartDistance   Where we are Starting at (for example what the wheel encoder should currently be reporting) 
     * @param aStartSpeed      The speed we are to start at (note can't be 0, else we wont go anywhwere to start moving)
     * @param aEndDistance     Where we are trying to get to
     * @param aEndSpeed        The speed we are trying to get to
     * @param source           The source information about, i.e. the encoder we are going to be monitoring
     * @param output           The output, i.e. the motor power we are going to be controlling to achieve the start-end speed profile desired
     */
    public AdjustSpeedAsTravelStartSpeedEndSpeedHelper(double aStartDistance, double aStartSpeed, 
    		                       double aEndDistance,   double aEndSpeed,
    		                       DoubleSupplier/*PIDSource*/ source//, DoubleConsumer/*PIDOutput*/ output
                                   ){
    		
    	m_startDistance          = aStartDistance; // start position, in distance units for example inches or Degrees of rotation
    	m_startSpeed             = aStartSpeed;    // start speed, in distance units for example inches or RevolutionsPerMin
    	    
    	m_endDistance            = aEndDistance; // end postion, in distance units for example inches or Degrees of rotation
    	m_endSpeed               = aEndSpeed; // end speed, in distance units for example inches or Degrees of rotation
    	
    	if(Math.abs(aStartDistance-aEndDistance) < 0.00000001) {
    		System.out.print("StartEndControllerHelper was started with the start and end being nearly the same, to avoid null pointer dont do this ");
    		m_ramp = 0;
    	} 
    	else {
    		m_ramp = (m_endSpeed - m_startSpeed )/ (m_endDistance-m_startDistance);
    	}    	
    }


	/**
     * Given the currentPosition (i.e. distance) this will return the current target speed 
     * @param currentMeasuredDistance // the current position, in distance units for example inches or Degrees-of-rotation
     * @return
     */
    public double getTargetSpeed(double currentMeasuredDistance){
    	
    	double distanceTraveledSoFar = currentMeasuredDistance - m_startDistance;
    	
    	double targetSpeed = m_startSpeed + m_ramp*distanceTraveledSoFar;
        
    	return targetSpeed;
    }
    

 

}
