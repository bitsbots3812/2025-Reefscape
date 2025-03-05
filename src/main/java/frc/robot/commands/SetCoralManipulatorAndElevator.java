// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.CoralManipulator.SetCoralManipulator;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetCoralManipulatorAndElevator extends ParallelCommandGroup {
  /** Creates a new SetCoralManipulatorAndElevator. */
  public SetCoralManipulatorAndElevator(double coralManipulatorSetpoint, double elevatorSetpoint, CoralManipulator coralManipulator, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetCoralManipulator(coralManipulatorSetpoint, coralManipulator),
      new SequentialCommandGroup(
        Commands.waitUntil(() -> coralManipulator.getAngle() >= 90 - CoralManipulatorConstants.ANGLE_SETPOINT_TOLERANCE),
        new SetElevator(elevatorSetpoint, elevator)
      )
    );
  }
}
