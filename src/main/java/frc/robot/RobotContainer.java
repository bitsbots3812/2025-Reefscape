// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.commands.Drivetrain.DefaultDrive;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.common.AxisSupplier;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.DriverStationLEDSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PDP;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  public enum RobotMode {
    CORAL("Coral"),
    ALGAE("Algae"),
    CLIMB("Climb");

    String displayName;

    RobotMode(String displayName) {
      this.displayName = displayName;
    }
    
  }

  RobotMode currentMode = RobotMode.CORAL;

  Trigger ModalBind(BooleanSupplier input, RobotMode activeMode) {

    return new Trigger(() -> {return input.getAsBoolean() && (activeMode == currentMode);});

  }

  PDP pdp = new PDP();
  AlgaeManipulator algaeManipulator = new AlgaeManipulator();
  CoralManipulator coralManipulator = new CoralManipulator();
  Elevator elevator = new Elevator(() -> pdp.getCurrent(PDPConstants.LEFT_ELEVATOR_MOTOR_PDP_PORT), () -> pdp.getCurrent(PDPConstants.RIGHT_ELEVATOR_MOTOR_PDP_PORT));
  Drivetrain driveTrain = new Drivetrain();
  DriverStationLEDSubsystem driverStationLEDSubsystem = new DriverStationLEDSubsystem();
  VisionSubsystem vision = new VisionSubsystem();

  SendableChooser<Command> autoChooser;

  CommandJoystick stick  = new CommandJoystick(0);
  CommandXboxController controller = new CommandXboxController(1);
  //GenericHID keyboard = new GenericHID(2);

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {

    //Driver Binds
    driveTrain.setDefaultCommand(new DefaultDrive(driveTrain, new AxisSupplier(stick.getHID(), 1, 2.0, 0.01, true), new AxisSupplier(stick.getHID(), 0, 2.0, 0.01, true)));
    //Manipulator Binds
    controller.start().onTrue(new InstantCommand(elevator::autoHome, elevator).until(() -> {return elevator.getState() != Elevator.ElevatorState.HOMING;}));
    controller.y().onTrue(new SetCoralManipulator());
    //Keyboard Binds
    
  }

  private void configureAutoCommands() {
    
    //Register named commands here

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Selector", autoChooser);
  }

  public void onEnable() {
    algaeManipulator.enable();
    coralManipulator.enable();
    elevator.enable();
  }

  public void onDisable() {
    algaeManipulator.disable();
    coralManipulator.disable();
    elevator.disable();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
