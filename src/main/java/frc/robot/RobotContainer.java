// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeManipulatorConstants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.commands.AlgaeManipulator.AlgaeManipulatorDefaultCommand;
import frc.robot.commands.AlgaeManipulator.SetAlgaeManipulator;
import frc.robot.commands.CoralManipulator.CoralManipulatorDefaultCommand;
import frc.robot.commands.CoralManipulator.SetCoralManipulator;
import frc.robot.commands.Drivetrain.DefaultDrive;
import frc.robot.commands.Elevator.ElevatorDefaultCommand;
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
    ALGAE("Algae");

    String displayName;

    RobotMode(String displayName) {
      this.displayName = displayName;
    }
    
  }

  RobotMode currentMode = RobotMode.CORAL;

  Trigger ModalBind(BooleanSupplier input, RobotMode activeMode) {

    return new Trigger(() -> {return input.getAsBoolean() && (activeMode == currentMode);});

  }

  DoubleSupplier ModalAnalogBind(DoubleSupplier input, RobotMode activeMode, double defaultValue) {
    return new DoubleSupplier() {
      @Override
      public double getAsDouble() {
          return currentMode == activeMode ? input.getAsDouble() : defaultValue;
      }
    };
  }

  DoubleSupplier ModalAnalogBind(DoubleSupplier input, RobotMode activeMode) {
    return ModalAnalogBind(input, activeMode, 0);
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

  boolean drivingReversed = false;

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {

    //Driver Binds
    //======================================================
    driveTrain.setDefaultCommand(new DefaultDrive(driveTrain, new AxisSupplier(stick.getHID(), 1, 2.0, 0.01, true), 
                                 new AxisSupplier(stick.getHID(), 0, 2.0, 0.01, true),
                                 () -> {return drivingReversed;}
      )
    );

    stick.button(2).onTrue(new InstantCommand(() -> {drivingReversed = !drivingReversed;}));
    
    ModalBind(stick.button(1), RobotMode.CORAL).whileTrue(
      new InstantCommand(() -> coralManipulator.setIntake(CoralManipulatorConstants.DEFAULT_INTAKE_OUT_SPEED), coralManipulator).repeatedly()
      .finallyDo(() -> coralManipulator.setIntake(0)));

    ModalBind(stick.button(1), RobotMode.ALGAE).whileTrue(
      new InstantCommand(() -> algaeManipulator.setIntakeMotor(AlgaeManipulatorConstants.DEFAULT_INTAKE_OUT_SPEED), algaeManipulator).repeatedly()
      .finallyDo(() -> algaeManipulator.setIntakeMotor(0.0)));

    
    //Manipulator Binds
    //========================================================
    algaeManipulator.setDefaultCommand(
      new AlgaeManipulatorDefaultCommand(ModalAnalogBind(new AxisSupplier(controller::getLeftY, 1, 0, true), RobotMode.ALGAE),
                                         ModalBind(controller.povDown(), RobotMode.ALGAE),
                                         ModalBind(controller.povUp(),  RobotMode.ALGAE),
                                         algaeManipulator
      )
    );

    coralManipulator.setDefaultCommand(
      new CoralManipulatorDefaultCommand(ModalAnalogBind(new AxisSupplier(controller::getLeftY, 1, 0, true), RobotMode.ALGAE),
                                         ModalBind(controller.povRight(), RobotMode.CORAL), 
                                         ModalBind(controller.povLeft(),  RobotMode.CORAL),
                                         coralManipulator
      )
    );

    elevator.setDefaultCommand(new ElevatorDefaultCommand(ModalAnalogBind(new AxisSupplier(controller::getRightY, 1, 0, true), RobotMode.CORAL), elevator));

    //Elevator and Coral Manipulator Setpoints
    //Home
    ModalBind(controller.a(), RobotMode.CORAL).onTrue(
      new ParallelCommandGroup(
        new SetCoralManipulator(CoralManipulatorConstants.SETPOINT_HOME_DEG, coralManipulator),
        new SequentialCommandGroup(
          Commands.waitUntil(() -> coralManipulator.getAngle() >= 90 - CoralManipulatorConstants.ANGLE_SETPOINT_TOLERANCE),
          new SetElevator(ElevatorConstants.SETPOINT_HOME, elevator)
        )
      )
    );

    //Trough
    ModalBind(controller.a(), RobotMode.CORAL).onTrue(
      new ParallelCommandGroup(
        new SetCoralManipulator(CoralManipulatorConstants.SETPOINT_TROUGH_DEG, coralManipulator),
        new SequentialCommandGroup(
          Commands.waitUntil(() -> coralManipulator.getAngle() >= 90 - CoralManipulatorConstants.ANGLE_SETPOINT_TOLERANCE),
          new SetElevator(ElevatorConstants.SETPOINT_HOME, elevator)
        )
      )
    );

    //L2
    ModalBind(controller.a(), RobotMode.CORAL).onTrue(
      new ParallelCommandGroup(
        new SetCoralManipulator(CoralManipulatorConstants.SETPOINT_REEF_DEG, coralManipulator),
        new SequentialCommandGroup(
          Commands.waitUntil(() -> coralManipulator.getAngle() >= 90 - CoralManipulatorConstants.ANGLE_SETPOINT_TOLERANCE),
          new SetElevator(ElevatorConstants.SETPOINT_L2, elevator)
        )
      )
    );


    //L3
    ModalBind(controller.a(), RobotMode.CORAL).onTrue(
      new ParallelCommandGroup(
        new SetCoralManipulator(CoralManipulatorConstants.SETPOINT_REEF_DEG, coralManipulator),
        new SequentialCommandGroup(
          Commands.waitUntil(() -> coralManipulator.getAngle() >= 90 - CoralManipulatorConstants.ANGLE_SETPOINT_TOLERANCE),
          new SetElevator(ElevatorConstants.SETPOINT_L3, elevator)
        )
      )
    );


    //Coral Station
    ModalBind(controller.a(), RobotMode.CORAL).onTrue(
      new ParallelCommandGroup(
        new SetCoralManipulator(CoralManipulatorConstants.SETPOINT_STATION_DEG, coralManipulator),
        new SequentialCommandGroup(
          Commands.waitUntil(() -> coralManipulator.getAngle() >= 90 - CoralManipulatorConstants.ANGLE_SETPOINT_TOLERANCE),
          new SetElevator(ElevatorConstants.SETPOINT_STATION, elevator)
        )
      )
    );



    //Algae Manipulator Setpoints
    //Home
    ModalBind(controller.y(), RobotMode.ALGAE).onTrue(new SetAlgaeManipulator(AlgaeManipulatorConstants.SETPOINT_HOME_DEG,   algaeManipulator));
    ModalBind(controller.x(), RobotMode.ALGAE).onTrue(new SetAlgaeManipulator(AlgaeManipulatorConstants.SETPOINT_INTAKE_DEG, algaeManipulator));
    ModalBind(controller.a(), RobotMode.ALGAE).onTrue(new SetAlgaeManipulator(AlgaeManipulatorConstants.SETPOINT_CLIMB_DEG,  algaeManipulator));

    

    controller.start().onTrue(new InstantCommand(elevator::autoHome, elevator).until(() -> {return elevator.getState() != Elevator.ElevatorState.HOMING;}));
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
