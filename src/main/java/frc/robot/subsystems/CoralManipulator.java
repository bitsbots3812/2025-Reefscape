// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.common.ArmController;
import frc.robot.common.ArmController.AngleControlState;
import frc.robot.common.EncoderVelocityTracker;

public class CoralManipulator extends SubsystemBase {

  //Initializing hardware & other stuff
  private VictorSPX centralTiltMotor = new VictorSPX(CoralManipulatorConstants.TILT_MOTOR_ID);
  private VictorSPX intakeMotorLeft = new VictorSPX(CoralManipulatorConstants.INTAKE_MOTOR_ID_LEFT);
  private VictorSPX intakeMotorRight = new VictorSPX(CoralManipulatorConstants.INTAKE_MOTOR_ID_RIGHT);
  private DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(CoralManipulatorConstants.ENCODER_CHANNEL);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 coralColorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch coralColorMatcher = new ColorMatch();

  private final Color kWhiteTarget = new Color(CoralManipulatorConstants.redVal, 
  CoralManipulatorConstants.greenVal, 
  CoralManipulatorConstants.blueVal);

  private ArmController arm = new ArmController(
    //Functional Interfaces
    (double speed) -> {centralTiltMotor.set(ControlMode.PercentOutput, speed);},
    this::getRawAngle,
    this::getAngularVelocity,

    //Angle Information
    CoralManipulatorConstants.ABSOLUTE_ENCODER_OFFSET,
    CoralManipulatorConstants.ANGLE_SETPOINT_TOLERANCE,
    CoralManipulatorConstants.allowedAngleRange,
    
    //PID Gains
    CoralManipulatorConstants.PID_P,
    CoralManipulatorConstants.PID_I,
    CoralManipulatorConstants.PID_D,

    //Feedforward Gains
    CoralManipulatorConstants.FF_KS,
    CoralManipulatorConstants.FF_KG,
    CoralManipulatorConstants.FF_KV,

    //Motion profile constraints
    CoralManipulatorConstants.MAX_ANGULAR_VELOCITY,
    CoralManipulatorConstants.MAX_PROFILED_ANGULAR_ACCELERATION,

    //Display Strings
    "Coral Manipulator",

    //Angle Unit Selection
    CoralManipulatorConstants.angleUnit
  );

  EncoderVelocityTracker velocityTracker = new EncoderVelocityTracker(this::getRawAngle);

  /** Creates a new CoralManipulator. */
  public CoralManipulator() {

    coralColorMatcher.addColorMatch(kWhiteTarget);
    
  }
  
  public double getRawAngle() {
    return tiltEncoder.get() * 360;
  }

  public void setIntake(double bothMotorSpeed) {
    intakeMotorLeft.set(ControlMode.PercentOutput, bothMotorSpeed);
    intakeMotorRight.set(ControlMode.PercentOutput, bothMotorSpeed);
  }

  public void setIntake(double leftSpeed, double rightSpeed) {
    intakeMotorLeft.set(ControlMode.PercentOutput, leftSpeed);
    intakeMotorRight.set(ControlMode.PercentOutput, rightSpeed);
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
    SmartDashboard.putNumber("Tilt Encoder (Coral Manipulator): ", getRawAngle());

    //Update velocity computation
    velocityTracker.update();
  }
}
