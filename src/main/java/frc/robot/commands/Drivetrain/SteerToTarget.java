// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SteerToTarget extends Command {
  /** Creates a new SteerToTarget. */

  DoubleSupplier xSpeed;
  BooleanSupplier driveReversed;
  double angleSetpoint;
  DoubleSupplier angle;
  Drivetrain drivetrain;

  PIDController pid;

  public SteerToTarget(DoubleSupplier xSpeed, BooleanSupplier driveReversed, double angleSetpoint, DoubleSupplier angle, double KP, double KD, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.xSpeed = xSpeed;
    this.driveReversed = driveReversed;
    this.angleSetpoint = angleSetpoint;
    this.angle = angle;
    this.drivetrain = drivetrain;

    pid = new PIDController(KP, 0, KD);

    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveReversed.getAsBoolean()) {
      drivetrain.drive(-xSpeed.getAsDouble(), pid.calculate(angle.getAsDouble(), angleSetpoint));
    }
    else {
      drivetrain.drive(xSpeed.getAsDouble(), pid.calculate(angle.getAsDouble(), angleSetpoint));
    }
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
