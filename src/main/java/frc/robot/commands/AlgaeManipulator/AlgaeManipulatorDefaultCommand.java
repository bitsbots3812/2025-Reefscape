// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeManipulator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeManipulatorConstants;
import frc.robot.subsystems.AlgaeManipulator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeManipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeManipulatorDefaultCommand extends Command {
  /** Creates a new AlgaeManipulatorDefaultCommand. */
  DoubleSupplier angleControl;
  Trigger intake;
  AlgaeManipulator algaeManipulator;

  public AlgaeManipulatorDefaultCommand(DoubleSupplier angleControl, Trigger intake, AlgaeManipulator algaeManipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleControl = angleControl;
    this.intake = intake;
    this.algaeManipulator = algaeManipulator;

    addRequirements(algaeManipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getAsBoolean()) {
      algaeManipulator.setIntakeMotor(Constants.AlgaeManipulatorConstants.INTAKE_MOTOR_SPEED_PERCENT);
    }
    else {
      algaeManipulator.setIntakeMotor(0.0);
    }

    //Control Angle
    algaeManipulator.setPointLeft(algaeManipulator.getSetpointLeft());
    algaeManipulator.setPointRight(algaeManipulator.getSetpointRight());
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
