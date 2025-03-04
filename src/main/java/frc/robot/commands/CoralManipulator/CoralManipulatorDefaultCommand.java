// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralManipulator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.subsystems.CoralManipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralManipulatorDefaultCommand extends Command {
   /** Creates a new CoralManipulatorDefaultCommand. */
   DoubleSupplier angleControl;
   Trigger intake;
   CoralManipulator coralManipulator;
   Timer manualControlTimer;

  public CoralManipulatorDefaultCommand(DoubleSupplier angleControl, Trigger intake, CoralManipulator coralManipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleControl = angleControl;
    this.intake = intake;
    this.coralManipulator = coralManipulator;

    addRequirements(coralManipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manualControlTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getAsBoolean()) {
      coralManipulator.setIntake(Constants.CoralManipulatorConstants.INTAKE_MOTOR_SPEED_PERCENT);
    }
    else {
      coralManipulator.setIntake(0.0);
    }

    //Control Angle
    coralManipulator.setPoint(coralManipulator.getSetpoint() + angleControl.getAsDouble() * CoralManipulatorConstants.MANUAL_CONTROL_RATE_DEG_SEC * manualControlTimer.get());
    manualControlTimer.restart();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
