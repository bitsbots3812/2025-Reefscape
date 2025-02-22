// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDP extends SubsystemBase {
  /** Creates a new PDP. */

  private PowerDistribution robotPDP = new PowerDistribution(0, ModuleType.kCTRE);

  public double getCurrent(int motor_id) {
    return robotPDP.getCurrent(motor_id);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
