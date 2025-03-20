package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
//import com.techhounds.houndutil.houndlog.annotations.Log;
//import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 

//@LoggedObject
public class Intake extends SubsystemBase implements BaseIntake {
    //@Log
    private final SparkMax motor;

    private SparkMaxConfig motorConfig;

    public Intake() {
        motorConfig = new SparkMaxConfig();
        motorConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);

        motor = new SparkMax(54, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    
    public void setRollerVoltage(double voltage) {
        voltage = voltage*8;
        motor.setVoltage(voltage);
    }

    @Override
    public Command runRollersCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(3),
                () -> setRollerVoltage(0))
                .withName("intake.runRollers");
    }

    @Override
    public Command reverseRollersCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(-12),
                () -> setRollerVoltage(0))
                .withName("intake.reverseRollers");
    }
}