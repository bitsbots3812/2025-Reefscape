// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDrive extends Command {
  /** Creates a new DefaultDrive. */

  private Drivetrain drivetrain;
  private DoubleSupplier xSpeed; 
  private DoubleSupplier zRotation;
  private BooleanSupplier driveReversed;

  public DefaultDrive(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier driveReversed) {
    this.drivetrain = drivetrain;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    this.driveReversed = driveReversed;
    addRequirements(drivetrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (driveReversed.getAsBoolean())
      drivetrain.drive(-xSpeed.getAsDouble(), zRotation.getAsDouble());
    else
      drivetrain.drive(xSpeed.getAsDouble(), zRotation.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
