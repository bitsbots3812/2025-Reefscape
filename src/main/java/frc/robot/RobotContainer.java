// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PDPConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PDP;

public class RobotContainer {

  PDP pdp = new PDP();
  Elevator elevator = new Elevator(() -> pdp.getCurrent(PDPConstants.LEFT_ELEVATOR_MOTOR_PDP_PORT), () -> pdp.getCurrent(PDPConstants.RIGHT_ELEVATOR_MOTOR_PDP_PORT));
  

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
