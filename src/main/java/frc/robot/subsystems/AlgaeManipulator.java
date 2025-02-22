// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.AnalogEncoder;
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
  private VictorSPX centralTiltMotor = new VictorSPX(AlgaeManipulatorConstants.CENTRAL_TILT_MOTOR_ID);
  private VictorSPX centralIntakeMotor = new VictorSPX(AlgaeManipulatorConstants.CENTRAL_INTAKE_MOTOR_ID);
  private AnalogEncoder tiltEncoder = new AnalogEncoder(AlgaeManipulatorConstants.ENCODER_CHANNEL);
  private DigitalInput irSensor = new DigitalInput(AlgaeManipulatorConstants.DIGITALINPUT_ID);

  private ArmController arm = new ArmController(
    //Functional Interfaces
    (double speed) -> {centralTiltMotor.set(ControlMode.PercentOutput, speed);},
    this::getRawAngle,
    this::getAngularVelocity,

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
    "Algae Arm",

    //Angle Unit Selection
    AlgaeManipulatorConstants.angleUnit
  );

  EncoderVelocityTracker velocityTracker = new EncoderVelocityTracker(this::getRawAngle);

  /** Creates a new AlgaeManipulator. */
  public AlgaeManipulator() {
    
  }
  
  public double getRawAngle() {
    return tiltEncoder.get() * 360;
  }

  public double getAngle() {
    return arm.getAngle();
  }

  public double getAngularVelocity() {
    return velocityTracker.getVelocity();
  }

  public void setProfiled(double setpoint) {
    arm.setProfiled(setpoint); 
  }

  public boolean atSetPoint() {
    return arm.atSetpoint();
  }

  public AngleControlState getState() {
    return arm.getState();
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
    SmartDashboard.putNumber("Tilt Encoder (Algae Arms): ", getRawAngle());

    //Update velocity computation
    velocityTracker.update();
  }
}
