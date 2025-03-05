// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeManipulatorConstants;
import frc.robot.common.ArmController;
import frc.robot.common.ArmController.AngleControlState;
import frc.robot.common.EncoderVelocityTracker;

public class AlgaeManipulator extends SubsystemBase {
  
  //Initializing hardware & other stuff
  private VictorSPX leftTiltMotor = new VictorSPX(AlgaeManipulatorConstants.LEFT_TILT_MOTOR_ID);
  private VictorSPX rightTiltMotor = new VictorSPX(AlgaeManipulatorConstants.RIGHT_TILT_MOTOR_ID);
  private VictorSPX centralIntakeMotor = new VictorSPX(AlgaeManipulatorConstants.CENTRAL_INTAKE_MOTOR_ID);
  private AnalogEncoder leftTiltEncoder = new AnalogEncoder(new AnalogInput(AlgaeManipulatorConstants.LEFT_ENCODER_CHANNEL));
  private AnalogEncoder rightTiltEncoder = new AnalogEncoder(new AnalogInput(AlgaeManipulatorConstants.RIGHT_ENCODER_CHANNEL));
  private DigitalInput irSensor = new DigitalInput(AlgaeManipulatorConstants.DIGITALINPUT_ID);

  private ArmController armLeft = new ArmController(
    //Functional Interfaces
    (double speed) -> {leftTiltMotor.set(ControlMode.PercentOutput, speed);},
    this::getRawAngleLeft,
    this::getAngularVelocityLeft,

    //Angle Information
    AlgaeManipulatorConstants.ABSOLUTE_ENCODER_OFFSET,
    AlgaeManipulatorConstants.ANGLE_SETPOINT_TOLERANCE,
    AlgaeManipulatorConstants.allowedAngleRange,
    
    //PID Gains
    AlgaeManipulatorConstants.PID_P,
    AlgaeManipulatorConstants.PID_I,
    AlgaeManipulatorConstants.PID_D,

    //Feedforward Gains
    AlgaeManipulatorConstants.FF_KS,
    AlgaeManipulatorConstants.FF_KG,
    AlgaeManipulatorConstants.FF_KV,

    //Motion profile constraints
    AlgaeManipulatorConstants.MAX_ANGULAR_VELOCITY,
    AlgaeManipulatorConstants.MAX_PROFILED_ANGULAR_ACCELERATION,

    //Display Strings
    "Algae Arm Left",

    //Angle Unit Selection
    AlgaeManipulatorConstants.angleUnit
  );

  private ArmController armRight = new ArmController(
    //Functional Interfaces
    (double speed) -> {rightTiltMotor.set(ControlMode.PercentOutput, speed);},
    this::getRawAngleRight,
    this::getAngularVelocityRight,

    //Angle Information
    AlgaeManipulatorConstants.ABSOLUTE_ENCODER_OFFSET,
    AlgaeManipulatorConstants.ANGLE_SETPOINT_TOLERANCE,
    AlgaeManipulatorConstants.allowedAngleRange,
    
    //PID Gains
    AlgaeManipulatorConstants.PID_P,
    AlgaeManipulatorConstants.PID_I,
    AlgaeManipulatorConstants.PID_D,

    //Feedforward Gains
    AlgaeManipulatorConstants.FF_KS,
    AlgaeManipulatorConstants.FF_KG,
    AlgaeManipulatorConstants.FF_KV,

    //Motion profile constraints
    AlgaeManipulatorConstants.MAX_ANGULAR_VELOCITY,
    AlgaeManipulatorConstants.MAX_PROFILED_ANGULAR_ACCELERATION,

    //Display Strings
    "Algae Arm Right",

    //Angle Unit Selection
    AlgaeManipulatorConstants.angleUnit
  );

  EncoderVelocityTracker velocityTrackerLeft = new EncoderVelocityTracker(this::getRawAngleLeft);
  EncoderVelocityTracker velocityTrackerRight = new EncoderVelocityTracker(this::getRawAngleRight);

  /** Creates a new AlgaeManipulator. */
  public AlgaeManipulator() {

    leftTiltMotor     .setInverted(AlgaeManipulatorConstants.LEFT_TILT_MOTOR_INVERTED);
    rightTiltMotor    .setInverted(AlgaeManipulatorConstants.RIGHT_TILT_MOTOR_INVERTED);
    centralIntakeMotor.setInverted(AlgaeManipulatorConstants.INTAKE_MOTOR_INVERTED);

    leftTiltMotor     .setNeutralMode(NeutralMode.Brake);
    rightTiltMotor    .setNeutralMode(NeutralMode.Brake);
    centralIntakeMotor.setNeutralMode(NeutralMode.Brake);

    leftTiltEncoder.setInverted(AlgaeManipulatorConstants.LEFT_TILT_ENCODER_INVERTED);
    rightTiltEncoder.setInverted(AlgaeManipulatorConstants.RIGHT_TILT_ENCODER_INVERTED);

  }
  
  public double getRawAngleLeft() {
    return leftTiltEncoder.get() * 360;
  }

  public double getRawAngleRight() {
    return rightTiltEncoder.get() * 360;
  }

  public double getAngleLeft() {
    return armLeft.getAngle();
  }
  
  public double getAngleRight() {
    return armRight.getAngle();
  }

  public double getAngularVelocityLeft() {
    return velocityTrackerLeft.getVelocity();
  }

  public double getAngularVelocityRight() {
    return velocityTrackerRight.getVelocity();
  }

  public boolean atSetPoint() {
    return armLeft.atSetpoint() && armRight.atSetpoint();
  }

  public void setPoint(double setpoint) {
    armLeft.setAngle(setpoint);
    armRight.setAngle(setpoint);
  }

  public void setProfiled(double setpoint) {
    armLeft.setProfiled(setpoint);
    armRight.setProfiled(setpoint);
  }

  public double getSetpoint() {
    return armLeft.getSetpoint();
  }

  public AngleControlState getStateLeft() {
    return armLeft.getState();
  }

  public AngleControlState getStateRight() {
    return armRight.getState();
  }

  public void enable() {
    armLeft.enable();
    armRight.enable();
  }

  public void disable() {
    armLeft.disable();
    armRight.disable();
  }

  public void setIntakeMotor(Double speed) {
    centralIntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean isLoaded() {
    return !irSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Tilt Encoder (Algae Arm Left): ", getRawAngleLeft());
    SmartDashboard.putNumber("Tilt Encoder (Algae Arm Right): ", getRawAngleRight());

    //Update velocity computation
    velocityTrackerLeft.update();
    velocityTrackerRight.update();
  }
}
