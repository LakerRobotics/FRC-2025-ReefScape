package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.techhounds.houndutil.houndlog.annotations.Log;
//import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RustConstants.Climber.*;

//@LoggedObject
public class Climber extends SubsystemBase {
   // @Log
    private final SparkMax motor;
    private final SparkMax motor2;
    private final SparkMax motor3;


    private SparkMaxConfig motorConfig;

   // @Log
    private boolean initialized;

    public Climber() {
        motorConfig = new SparkMaxConfig();
        motorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);
        motorConfig.encoder
                .positionConversionFactor(ENCODER_ROTATIONS_TO_METERS)
                .velocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0);

        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        motor2 = new SparkMax(MOTOR_ID2, MotorType.kBrushless);
        motor3 = new SparkMax(MOTOR_ID3, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor3.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor.getEncoder().setPosition(0);
    }

    public boolean getInitialized() {
        return initialized;
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -4, 4);
        System.out.print("reached climber voltage"+voltage);
        motor.setVoltage(voltage);
        motor2.setVoltage(voltage);
        motor3.setVoltage(voltage);
    }

    public Command winchUpCommand() {
        return Commands.startEnd(
                () -> setVoltage(12),
                () -> setVoltage(0))
                .withName("climber.winchUp");
    }

    public Command winchDownCommand() {
        return Commands.startEnd(
                () -> setVoltage(12),
                () -> setVoltage(0))
                .withName("climber.winchDown");
    }

    public void resetPosition() {
        motor.getEncoder().setPosition(0);
        initialized = true;
    }

    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("arm.resetPosition");
    }
}