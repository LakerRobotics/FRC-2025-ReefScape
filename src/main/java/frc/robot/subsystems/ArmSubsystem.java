// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.spark.SparkBase.IdleMode;
//import com.revrobotics.CANSparkBase.SoftLimitDirection;
//import com.revrobotics.CANSparkLowLevel.MotorType;

import javax.management.modelmbean.ModelMBean;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkMaxLimitSwitch;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;
import frc.robot.Constants2023;
import frc.robot.commands.ArmHomePosition;
import frc.robot.commands.ArmJoystickControl;

public class ArmSubsystem extends SubsystemBase {
  SparkMaxConfig config_m_motor;
  // Enumeration for control modes
     private enum MOD {
      POSITION_CONTROL,
      POWER_CONTROL,
      VELOCITY_CONTROL
  }
  // Default control mode
  private MOD mode = MOD.POSITION_CONTROL;

  private SparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkClosedLoopController m_controller;
  private double m_setpoint;

  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;

  private TrapezoidProfile.State m_targetState;
  private double m_feedforward;
  private double m_manualValue;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    // create a new SPARK MAX and configure it
config_m_motor = new SparkMaxConfig();

config_m_motor
    .inverted(true)
    .idleMode(IdleMode.kBrake);
config_m_motor.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
config_m_motor.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
    

    //config_m_motor.()//m_motor = new CANSparkMax(Constants.Arm.kArmCanId, MotorType.kBrushless);
    config_m_motor.inverted(false);//m_motor.setInverted(false);
    config_m_motor.smartCurrentLimit(Constants.Arm.kCurrentLimit);//setSmartCurrentLimit(Constants.Arm.kCurrentLimit);
    config_m_motor.idleMode(SparkBaseConfig.IdleMode.kBrake);//m_motor.setIdleMode(IdleMode.kBrake);
    config_m_motor.softLimit.forwardSoftLimitEnabled(true);//m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    config_m_motor.softLimit.reverseSoftLimitEnabled(true);//m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    config_m_motor.softLimit.forwardSoftLimit(Constants.Arm.kSoftLimitForward);//m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.kSoftLimitForward);
    config_m_motor.softLimit.reverseSoftLimit(Constants.Arm.kSoftLimitReverse);//m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.kSoftLimitReverse);
    m_motor.configure(config_m_motor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // set up the motor encoder including conversion factors to convert to radians and radians per second for position and velocity
    m_encoder = m_motor.getEncoder();
    //TODO 
    //m_encoder.positionConversionFactor();//setPositionConversionFactor(Constants.Arm.kPositionFactor);
    //m_encoder.setVelocityConversionFactor(Constants.Arm.kVelocityFactor);
    m_encoder.setPosition(0.0);

    m_controller = m_motor.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.Arm.kArmPositionGains);

    //m_motor.burnFlash();

    m_setpoint = Constants.Arm.kHomePosition;

    m_timer = new Timer();
    m_timer.start();

    updateMotionProfile();

    setDefaultCommand(new ArmHomePosition(this));
    //set a default so auton does not crash for Auton
          m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);

  }
  
  // Set control mode to POSITION_CONTROL
  private void setControlModePosition() {
    config_m_motor.softLimit.forwardSoftLimitEnabled(true);
    //m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    config_m_motor.softLimit.reverseSoftLimitEnabled(true);//m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_motor.configure(config_m_motor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      this.mode = MOD.POSITION_CONTROL;
  }

  // Method to set control mode to VELOCITY_CONTROL
  private void setControlModeVelocity() {
    config_m_motor.softLimit.forwardSoftLimitEnabled(false);
    config_m_motor.softLimit.reverseSoftLimitEnabled(false);
    m_motor.configure(config_m_motor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.mode = MOD.VELOCITY_CONTROL;
  }

  // Set control mode to POWER_CONTROL
  private void setControlModePower() {
    config_m_motor.softLimit.forwardSoftLimitEnabled(false);
    config_m_motor.softLimit.reverseSoftLimitEnabled(false);
    m_motor.configure(config_m_motor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //  m_motor.enableSoftLimit(SoftLimitDirection.kForward, false);
  // m_motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    this.mode = MOD.POWER_CONTROL;
  }
  private void setSoftLimtBasedOnCurrentLocation(){
    config_m_motor.softLimit.forwardSoftLimit(Constants.Arm.kSoftLimitForward); 
    config_m_motor.softLimit.reverseSoftLimit(Constants.Arm.kSoftLimitReverse);
    m_motor.configure(config_m_motor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.kSoftLimitForward);
  //m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.kSoftLimitReverse);
    setControlModePosition();
  }
  public void setCanSparkEncoderToZero() {
    m_encoder.setPosition(0.0);
}
  /**
   * Sets the target position and updates the motion profile if the target position changed.
   * @param _setpoint The new target position in radians.
  */
  public void setTargetPosition(double _setpoint) {
    setControlModePosition();
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    }
  }
  // return if the top (forward), limit switch is pressed
  public boolean isLimitSwitchTopPressed(){
    return m_motor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
  }
  // return if the bottom (backward), limit switch is pressed
  public boolean isLimitSwitchBottomPressed(){
    return m_motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
  }

  /**Update the motion profile variables based on the current setpoint and the pre-configured motion constraints.*/
  private void updateMotionProfile() {
    m_startState = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
    m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint);
    m_timer.reset();
  }

  /**
   * Drives the arm to a position using a trapezoidal motion profile.
   * This function is usually wrapped in a {@code RunCommand} which runs it repeatedly while the command is active.
   * <p>
   * This function updates the motor position control loop using a setpoint from the trapezoidal motion profile.
   * The target position is the last set position with {@code setTargetPosition}.
   */
  public void runAutomatic() {
    setControlModePosition();
    double elapsedTime = m_timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    } else {
      m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
    }

    m_feedforward =
        Constants.Arm.kArmFeedforward.calculate(
            m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset, m_targetState.velocity);
    m_controller.setReference(
        m_targetState.position, SparkMax.ControlType.kPosition, 0, m_feedforward);
  }

  /**
   * Drives the arm using the provided power value (usually from a joystick).
   * This also adds in the feedforward value which can help counteract gravity.
   * @param _power The motor power to apply.
   */
  public void runManual(double _power) {
    setControlModePower();
    // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
    // passively
    m_setpoint = m_encoder.getPosition();
    updateMotionProfile();
    // update the feedforward variable with the newly zero target velocity
    m_feedforward =
        Constants.Arm.kArmFeedforward.calculate(
            m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset, m_targetState.velocity);
    // set the power of the motor
    m_motor.set(_power + (m_feedforward / 12.0));
    m_manualValue = _power; // this variable is only used for logging or debugging if needed
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
  }

  public double getArmPosition() {
    return m_encoder.getPosition();
  }
}