package frc.robot.subsystems.utilities;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Encoder; 

public class EncoderDistenceAsDouble implements DoubleSupplier { 
    Encoder m_encoder; 
    public EncoderDistenceAsDouble(Encoder anEncoder){
        m_encoder = anEncoder;
    }
    

    @Override
    public double getAsDouble() {
        return m_encoder.getDistance();
    }
}
