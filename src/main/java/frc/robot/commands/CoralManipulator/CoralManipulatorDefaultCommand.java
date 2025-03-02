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

  DoubleSupplier angleControl;
  Trigger intakeIn;
  Trigger intakeOut;
  Trigger driverTrigger;
  CoralManipulator coralManipulator;
  Timer angleControlTimer = new Timer();
  
  /** Creates a new CoralManipulatorDefaultCommand. */
  public CoralManipulatorDefaultCommand(DoubleSupplier angleControl, Trigger intakeIn, Trigger intakeOut, Trigger driverTrigger, CoralManipulator coralManipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
     this.angleControl = angleControl;
    this.intakeIn = intakeIn;
    this.intakeOut = intakeOut;
    this.driverTrigger = driverTrigger;
    this.coralManipulator = coralManipulator;

    addRequirements(coralManipulator);

  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    angleControlTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    
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
