package frc.robot.subsystems.utilities;
//import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author richard.topolewski  
 *
 */
public class AdjustSpeedAsTravelMotionControlHelperWithMinSpeed  extends AdjustSpeedAsTravelHelper {
	double m_targetDistance         = 0.0d; // in distance units for example inches or Degrees of rotation
	double m_rampUpRampDownDistance = 0.0d; // in distance units for example inches or Degrees of roation
    double m_runningSpeed           = 0.0d; // distance (e.g. inchs or Degrees of rotation) over seconds
    double m_slowestSpeed           = 0.0d; // distance (e.g. inchs or Degrees of rotation) over seconds

    double m_currentMeasuredDistance  = 0.0d;
    double m_initialMeasuredDistance  = 0.0d;

    boolean isGoingForward = true;
 
 //  	public double percentDeadZoneOverride = 0.15;//enter portion of 1 (e.g. .1 for 10%)
   	
   
	/**
     * This helper class just takes a target distance and will provide a motion control speed to get to that target
     * @param aTargetDistance   Where we are trying to get to
     * @param aRampUpRampDownDistance  Allows control of how fast we accelerate and decelerate
     * @param aRunningSpeed   the speed we want to travel most of the time, except for ramp up and ramp downb
     * @param aInitialMeasuredDistance  so we know where we started from for the ramp up
     */
    public AdjustSpeedAsTravelMotionControlHelperWithMinSpeed(double targetDistance, double rampUpRampDownDistance, double runningSpeed, 
    		            double initialMeasuredDistance,
                        DoubleSupplier source,//, DoubleConsumer output // used in abstract base class
                        double slowestSpeed
                        ){
    	m_targetDistance          = targetDistance;
    	m_rampUpRampDownDistance  = Math.abs(rampUpRampDownDistance);
    	m_runningSpeed            = runningSpeed;
    	m_initialMeasuredDistance = Math.abs(initialMeasuredDistance);
//    	m_output = output;
    	m_source = source;
        m_slowestSpeed            = slowestSpeed;
        if ((m_targetDistance - m_initialMeasuredDistance)<0){
            isGoingForward=false;
        }
        else{
            isGoingForward = true;
        }
    }

    /**
     * Given the currentPosition (i.e. distance) this will return the current target speed 
     * @param currentMeasuredDistance // the current position, in distance units for example inches or Degrees-of-rotation
     * @return
     */
    public double getTargetSpeed(double currentMeasuredDistance){
       double targetSpeed = m_slowestSpeed;       
       
       // get the    going in the right direction
       double gapEnd = m_targetDistance-currentMeasuredDistance;
       if(gapEnd == 0) 
       {
    	   targetSpeed = 0;
       }
       else 
       {
    	   targetSpeed = (gapEnd/Math.abs(gapEnd)) * m_runningSpeed; // This just applied +1 or -1 to get the sign right
       }
       SmartDashboard.putNumber("AdjustSpeedAsTravel_withMin_targetSpeed1",targetSpeed);       
       
       // Calculate the reduction to the speed if at the start
       double percentRampUp;
       double gapStart = currentMeasuredDistance-m_initialMeasuredDistance;
       if( Math.abs(gapStart) > m_rampUpRampDownDistance){
    	   // We are outside of the rampUp zone 
    	   percentRampUp = 1; //100%
       }
       else{
    	   // Are we right on top of the start point, if so, don't set motor to zero but some minimum number to get things to move
/* Redundent given the minimum speed overide now coded down below
           if( Math.abs(gapStart) < m_rampUpRampDownDistance*(percentDeadZoneOverride)){               
    	      percentRampUp = percentDeadZoneOverride ; //just to make sure it does not stay stuck at the start 
           }
           else{
               percentRampUp = Math.abs(gapStart)/m_rampUpRampDownDistance;
           }
*/
            percentRampUp = Math.abs(gapStart)/m_rampUpRampDownDistance;
        }
        SmartDashboard.putNumber("AdjustSpeedAsTravel_withMin_targetSpeed2",targetSpeed);       

       
       // Calculate reduction to the speed if we are at the end
       double rampDown = m_rampUpRampDownDistance;
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
       SmartDashboard.putNumber("AdjustSpeedAsTravel_withMin_targetSpeed3",targetSpeed);       

       // Minimum slowest speed overide
       if( Math.abs(targetSpeed)<Math.abs(m_slowestSpeed)){
           if (isGoingForward){
            targetSpeed = m_slowestSpeed;
           }else{
            targetSpeed = -m_slowestSpeed;
           }
       }
       SmartDashboard.putNumber("AdjustSpeedAsTravel_withMin_m_slowestSpeed",m_slowestSpeed);       
       SmartDashboard.putNumber("AdjustSpeedAsTravel_withMin_targetSpeed4",targetSpeed);       
       SmartDashboard.putNumber("AdjustSpeedAsTravel_withMin_percentRampUp", percentRampUp);
       SmartDashboard.putNumber("AdjustSpeedAsTravel_withMin_percentRampDown", percentRampDown);
       return targetSpeed;
    }

    
}
