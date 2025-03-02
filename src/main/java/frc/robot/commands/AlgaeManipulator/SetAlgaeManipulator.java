// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeManipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAlgaeManipulator extends Command {
  /** Creates a new SetAlgaeManipulator. */
  AlgaeManipulator algaeManipulator;
  double setpoint; 

  public SetAlgaeManipulator(double setpoint, AlgaeManipulator algaeManipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setpoint = setpoint;
    this.algaeManipulator = algaeManipulator;

    addRequirements(algaeManipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeManipulator.setProfiledLeft(setpoint);
    algaeManipulator.setProfiledRight(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeManipulator.atSetPointLeft() && algaeManipulator.atSetPointRight();
  }
}
