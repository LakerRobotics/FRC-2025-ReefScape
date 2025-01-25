package frc.robot.subsystems.utilities;

import edu.wpi.first.math.controller.PIDController;
//import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class AdjustSpeedAsTravelHelper {  

//	protected DoubleConsumer /*PIDOutput*/ m_output;
	protected DoubleSupplier /*PIDSource*/ m_source;
	PIDController regularPIDControl;

	public AdjustSpeedAsTravelHelper() {
		super();
	}

    abstract public double getTargetSpeed(double currentMeasuredDistance);

	protected PIDController getRegularPIDControl() {
		return regularPIDControl;
	}

	protected void setRegularPIDControl(PIDController regularPIDControl) {
		this.regularPIDControl = regularPIDControl;
	}

//	public DoubleConsumer /*PIDOutput*/ getM_output() {
//		return m_output;
//	}

	/**
	 * This returns the PIDSource wrapped so when called by the PIDController the motionControlHelper can
	 * adjust the target rate that the PIDController is trying to achieve
	 * @return
	 */
/*	public DoubleSupplier /*PIDSource*//* getM_source() {
//		return new wrapPIDInput(this, m_source);
		return m_source;
	}
*/

	/**
	 * Read the input(i.e. position) and calculate the speed for this position and put that in as the setPoint
	 */
/*	protected void adjustTargetSpeed() throws Exception {
		double currentSpeed = m_source.getAsDouble();
		// get the adjusted target speed, this is provided by the implementation class.
		double targetSpeed = getTargetSpeed(this.getMeasurment());
		
		SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed Measurement", this.getMeasurment());
//		this.getRegularPIDControl().setGoal(targetSpeed); 
		this.getRegularPIDControl().setSetpoint(targetSpeed);
		SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed targetSpeed", targetSpeed);
		
		SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed Gyro Rate", m_source.getAsDouble());
	}
*/
	public double getMeasurment() {
		
		SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed currentSpeed", m_source.getAsDouble());
		SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed targetSpeed", getTargetSpeed(m_source.getAsDouble()/*currentMeasuredDistance*/));
		
		return m_source.getAsDouble();
	}

	// Can this be eliminated? TODO Can this be eliminated
	class wrapPIDInput implements DoubleSupplier /*PIDSource*/ {

//	    private AdjustSpeedAsTravelHelper m_MCHelper;
	    private DoubleSupplier/*PIDSource*/ m_source; 

	    public wrapPIDInput(AdjustSpeedAsTravelHelper motionControlHelper, DoubleSupplier /*PIDSource*/ source, DoubleSupplier sourceRate) {
/*	        if (motionControlHelper == null) {
				System.out.print("AdjustSpeeAsTravelHelper. wrapPIDInput(..) Given AdjustSpeedAsTravelHelper was null");
	            throw new NullPointerException("Given AdjustSpeedAsTravelHelper was null");
	        }
	        else{
                m_MCHelper = motionControlHelper;            	
	        }
*/
	        if (source == null){
				System.out.print("AdjustSpeeAsTravelHelper. wrapPIDInput(..) Given DoubleSupplier source was null");
	            throw new NullPointerException("Given DoubleSupplier source was null");
	        }
	        else{
	            m_source = source;
	        }

	    }
	        
		@Override
	    //public double pidGet(){
		public double getAsDouble() {
				// have the controller set the target speed,
	        //TODO have WPI redo the PIDController so the calculate() method is protected so we wouldn't have to do this hack 
			//  if it were protected then we could override calculate() method and allow the target speed to be set ahead of calculation the new PID output
/*				try{
					m_MCHelper.adjustTargetSpeed();
				}
				catch (Exception e){
					System.out.println("MotionControl PIDSource BAD, likley not Gyro or Encoder or missing getMeasurement()");
					System.out.println(e);
				}
*/				// call the real PIDSource
	        	//return m_source.pidGet();
				return m_source.getAsDouble();
	        }
	    }
}