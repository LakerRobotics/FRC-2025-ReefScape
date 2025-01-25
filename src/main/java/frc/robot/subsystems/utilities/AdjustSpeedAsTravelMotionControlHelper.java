package frc.robot.subsystems.utilities;
//import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author richard.topolewski   
 *
 */
public class AdjustSpeedAsTravelMotionControlHelper extends AdjustSpeedAsTravelHelper {
	double m_targetDistance         = 0.0d; // in distance units for example inches or Degrees of rotation
	double m_rampUpRampDownDistance = 0.0d; // in distance units for example inches or Degrees of roation
    double m_runningSpeed           = 0.0d; // distance (e.g. inchs or Degrees of rotation) over seconds	
	
    double m_currentMeasuredDistance  = 0.0d;
    double m_initialMeasuredDistance  = 0.0d;
 
   	public double percentDeadZoneOverride = 0.15;//enter portion of 1 (e.g. .1 for 10%)
   	
   	private final double RAMP_MULTIPLIER = 4.0;
   
	/**
     * This helper class just takes a target distance and will provide a motion control speed to get to that target
     * @param aTargetDistance   Where we are trying to get to
     * @param aRampUpRampDownDistance  Allows control of how fast we accelerate and decelerate
     * @param aRunningSpeed   the speed we want to travel most of the time, except for ramp up and ramp down
     * @param aInitialMeasuredDistance  so we know where we started from for the ramp up
     */
    public AdjustSpeedAsTravelMotionControlHelper(double targetDistance, double rampUpRampDownDistance, double runningSpeed, 
    		            double initialMeasuredDistance,
                        DoubleSupplier source//, DoubleConsumer output // used in abstract base class
                        ){
    	m_targetDistance          = targetDistance;
    	m_rampUpRampDownDistance  = Math.abs(rampUpRampDownDistance);
    	m_runningSpeed            = runningSpeed;
    	m_initialMeasuredDistance = Math.abs(initialMeasuredDistance);
//    	m_output = output;
    	m_source = source;
    }

    /**
     * Given the currentPosition (i.e. distance) this will return the current target speed 
     * @param currentMeasuredDistance // the current position, in distance units for example inches or Degrees-of-rotation
     * @return
     */
    public double getTargetSpeed(double currentMeasuredDistance){
       double targetSpeed = 0.0d;       
       
       // get the    going in the right direction
       double gapEnd = m_targetDistance-currentMeasuredDistance;
       double rampDown = m_rampUpRampDownDistance * RAMP_MULTIPLIER;
       if(gapEnd == 0) 
       {
    	   targetSpeed = 0;
       }
       else 
       {
    	   targetSpeed = (gapEnd/Math.abs(gapEnd)) * m_runningSpeed; // This just applied +1 or -1 to get the sign right
       }
       
       // Calculate the reduction to the speed if at the start
       double percentRampUp;
       double gapStart = currentMeasuredDistance-m_initialMeasuredDistance;
       if( Math.abs(gapStart) > m_rampUpRampDownDistance){
    	   // We are outside of the rampUp zone 
    	   percentRampUp = 1; //100%
       }
       else{
    	   // Are we right on top of the start point, if so, don't set motor to zero but some minimum number to get things to move
           if( Math.abs(gapStart) < m_rampUpRampDownDistance*(percentDeadZoneOverride)){
    	      percentRampUp = percentDeadZoneOverride ; //just to make sure it does not stay stuck at the start 
           }
           else{
               percentRampUp = Math.abs(gapStart)/m_rampUpRampDownDistance;
           }
       }
       
       // Calculate reduction to the speed if we are at the end
       double percentRampDown = Math.abs(gapEnd)/rampDown;
       if (Math.abs(percentRampDown)>1)  percentRampDown = 1; // limit percent to 100%

       //Apply any speed reductions based on rampUp or rampDown.
       //System.out.println("fromStart="+gapStart+"("+percentRampUp+")   fromEnd="+gapEnd+"("+percentRampDown+")");
       // Removed this 4/12/2016 seemed to cause problem, if close the end, the start dead overide would kick in and make it overshoot the target       
       //if(Math.abs(gapStart)<Math.abs(gapEnd)){
       //targetSpeed = percentRampUp * targetSpeed;
       //}
       //else{
       // If we are near the end, then ramp down
       if (Math.abs(gapStart) < m_rampUpRampDownDistance) {
    	   targetSpeed = percentRampUp * targetSpeed;
       }
       else if(Math.abs(gapEnd) < rampDown){
    	   targetSpeed = percentRampDown * targetSpeed;
       }
       SmartDashboard.putNumber("targetSpeed",targetSpeed);       
       SmartDashboard.putNumber("getTargetSpeed MotionControlHelper", targetSpeed);
       return targetSpeed;
    }
    


    
 
}
