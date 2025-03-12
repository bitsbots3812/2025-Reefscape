// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeRemover;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.subsystems.AlgaeRemover;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeRemoverDefaultCommand extends Command {
   /** Creates a new AlgaeRemoverDefaultCommand. */
   DoubleSupplier angleControl;
   AlgaeRemover AlgaeRemover;
   Timer manualControlTimer = new Timer();

  public AlgaeRemoverDefaultCommand(DoubleSupplier angleControl, AlgaeRemover AlgaeRemover) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleControl = angleControl;
    this.AlgaeRemover = AlgaeRemover;

    addRequirements(AlgaeRemover);
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
    AlgaeRemover.setPoint(AlgaeRemover.getSetpoint() + angleControl.getAsDouble() * AlgaeRemoverConstants.MANUAL_CONTROL_RATE_DEG_SEC * manualControlTimer.get());
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