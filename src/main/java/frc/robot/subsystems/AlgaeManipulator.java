// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeManipulatorConstants;

public class AlgaeManipulator extends SubsystemBase {
  
  //Initializing hardware & other stuff
  private VictorSPX centralTiltMotor = new VictorSPX(AlgaeManipulatorConstants.CENTRAL_TILT_MOTOR_ID);
  private VictorSPX centralIntakeMotor = new VictorSPX(AlgaeManipulatorConstants.CENTRAL_INTAKE_MOTOR_ID);
  private DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(AlgaeManipulatorConstants.ENCODER_CHANNEL);
  private DigitalInput irSensor = new DigitalInput(AlgaeManipulatorConstants.DIGITALINPUT_ID);
  private PIDController tiltMotorPID = new PIDController(AlgaeManipulatorConstants.PID_P, AlgaeManipulatorConstants.PID_I, AlgaeManipulatorConstants.PID_D);
  private double setpoint = 0;

  /** Creates a new AlgaeManipulator. */
  public AlgaeManipulator() {
    // what to do here?
  }

  public double getRotation() {
    return tiltEncoder.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Tilt Encoder (Algae Arms): ", getRotation());

    centralTiltMotor.set(ControlMode.PercentOutput, tiltMotorPID.calculate(getRotation(), setpoint));
  }
}
