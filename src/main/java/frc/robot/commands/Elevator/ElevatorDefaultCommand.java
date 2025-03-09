// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDefaultCommand extends Command {
  DoubleSupplier heightControl;
  Timer manualControlTimer = new Timer();
  Elevator elevator;

  public ElevatorDefaultCommand(DoubleSupplier heightControl, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.heightControl = heightControl;
    this.elevator = elevator;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manualControlTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Control Angle
    elevator.setPosition(elevator.getSetpoint() + heightControl.getAsDouble() * ElevatorConstants.MANUAL_CONTROL_RATE_METER_SEC * manualControlTimer.get());
    manualControlTimer.restart();
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
