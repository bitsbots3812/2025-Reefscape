// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  VictorSPX motor = new VictorSPX(ClimberConstants.MOTOR_ID);

  DigitalInput limitSwitch = new DigitalInput(ClimberConstants.LIMIT_SWITCH_PORT);

  private double speed = 0;

  public Climber() {
    motor.setInverted(ClimberConstants.MOTOR_INVERTED);
  }

  public void set(double speed) {
    motor.set(ControlMode.PercentOutput, MathUtil.clamp(speed, 0, 1));
  }

  @Override
  public void periodic() {}
}
