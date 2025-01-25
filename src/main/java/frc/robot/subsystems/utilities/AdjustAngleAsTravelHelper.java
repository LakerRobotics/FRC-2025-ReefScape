package frc.robot.subsystems.utilities;

public class AdjustAngleAsTravelHelper {

	double m_startDistance;
	double m_cicleRadius;
	double m_startAngle;	 
	double m_targetAngle;
	double m_distanceOnArch; 
/**
 * 
 * @param startDistance provide the current distance (typically in inches)
 * @param circleRadius  inches
 * @param currentAngle  Degrees
 * @param targetAngle   Degrees
 */
	public AdjustAngleAsTravelHelper(double startDistance, double circleRadius, double currentAngle, double targetAngle) {
		        //Calculate length of arc on the cirle we are to traverse
				double archAngle = targetAngle-currentAngle;
		
				double percentOfCircle = archAngle/360;
			 		
				// Calculate distance based on percent of diameter traveled
				m_distanceOnArch = circleRadius*java.lang.Math.PI*percentOfCircle;
		
	}

    public double getTargetAngle(double currentMeasuredDistance){
		double percentTraveled = (currentMeasuredDistance - m_startDistance)/m_distanceOnArch;
		double archAngle = m_targetAngle -m_startAngle;
		double angleTraveledSoFar = archAngle * percentTraveled;
		double angleShouldBeAt = m_startAngle + angleTraveledSoFar;
		return angleShouldBeAt;
	};

	public double getLengthOfArc(){
		return m_distanceOnArch;
	};


}