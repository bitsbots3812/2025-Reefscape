// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.common.ArmController;
import frc.robot.common.ArmController.AngleControlState;
import frc.robot.common.EncoderVelocityTracker;

public class AlgaeRemover extends SubsystemBase {

  private VictorSPX mainMotor = new VictorSPX(AlgaeRemoverConstants.MAIN_MOTOR_ID);
  private AnalogEncoder mainAnalogEncoder = new AnalogEncoder(AlgaeRemoverConstants.ANALOG_ENCODER_ID);
  
  private ArmController arm = new ArmController(
    //Functional Interfaces
    (double speed) -> {mainMotor.set(ControlMode.PercentOutput, speed);},
    this::getRawAngle,
    () -> 0,

    //Angle Information
    AlgaeRemoverConstants.ANALOG_ENCODER_OFFSET,
    AlgaeRemoverConstants.ANGLE_SETPOINT_TOLERANCE,
    AlgaeRemoverConstants.allowedAngleRange,
    //PID Gains
    AlgaeRemoverConstants.PID_P,
    AlgaeRemoverConstants.PID_I,
    AlgaeRemoverConstants.PID_D,

    //Feedforward Gains
    AlgaeRemoverConstants.FF_KS,
    AlgaeRemoverConstants.FF_KG,
    AlgaeRemoverConstants.FF_KV,

    //Motion profile constraints
    AlgaeRemoverConstants.MAX_ANGULAR_VELOCITY,
    AlgaeRemoverConstants.MAX_PROFILED_ANGULAR_ACCELERATION,

    //Display Strings
    "Algae Kicker",

    //Angle Unit Selection
    AlgaeRemoverConstants.angleUnit
  );

  /** Creates a new AlgaeRemover. */
  public AlgaeRemover() {
    mainMotor.setInverted(AlgaeRemoverConstants.MAIN_MOTOR_INVERTED);
    mainMotor.setNeutralMode(NeutralMode.Brake);

    mainAnalogEncoder.setInverted(AlgaeRemoverConstants.ENCODER_INVERTED);
  }

  public double getRawAngle() {
    return mainAnalogEncoder.get() * 360;
  }

  public double getAngle() {
    return arm.getAngle();
  }

  public void setProfiled(double setpoint) {
    arm.setProfiled(setpoint); 
  }

  public boolean atSetPoint() {
    return arm.atSetpoint();
  }

  public double getSetpoint() {
    return arm.getSetpoint();
  }

  public AngleControlState getState() {
    return arm.getState();
  }

  public void setPoint(double setpoint) {
    arm.setAngle(setpoint);
  }

  public void enable() {
    arm.enable();
  }

  public void disable() {
    arm.disable();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.execute();

  }
}
