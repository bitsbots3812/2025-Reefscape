// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.Constants.VisionSubsystemConstants;
import frc.robot.commands.CoralandElevatorSetpointSequence;
import frc.robot.commands.SetCoralManipulatorAndElevator;
import frc.robot.commands.AlgaeRemover.SetAlgaeRemover;
import frc.robot.commands.CoralManipulator.CoralManipulatorDefaultCommand;
import frc.robot.commands.CoralManipulator.SetCoralManipulator;
import frc.robot.commands.Drivetrain.DefaultDrive;
import frc.robot.commands.Drivetrain.SteerToTarget;
import frc.robot.commands.Elevator.ElevatorDefaultCommand;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.common.AxisSupplier;
import frc.robot.common.ArmController.AngleControlState;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.DriverStationLEDSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PDP;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriverStationLEDSubsystem.LEDEffect;

public class RobotContainer {

  //Instantiate Subsystems
  //=======================================
  PDP pdp = new PDP();
  CoralManipulator coralManipulator = new CoralManipulator();
  AlgaeRemover algaeRemover = new AlgaeRemover();
  Elevator elevator = new Elevator(() -> pdp.getCurrent(PDPConstants.LEFT_ELEVATOR_MOTOR_PDP_PORT), () -> pdp.getCurrent(PDPConstants.RIGHT_ELEVATOR_MOTOR_PDP_PORT));
  Drivetrain driveTrain = new Drivetrain();
  DriverStationLEDSubsystem driverStationLEDSubsystem = new DriverStationLEDSubsystem();
  VisionSubsystem vision = new VisionSubsystem();
  DriverStationLEDSubsystem leds = new DriverStationLEDSubsystem();

  SendableChooser<Command> autoChooser;

  CommandJoystick stick  = new CommandJoystick(0);
  CommandXboxController controller = new CommandXboxController(1);
  //GenericHID keyboard = new GenericHID(2);

  boolean drivingReversed = false;

  public RobotContainer() {

    configureBindings();

    configureAutoCommands();

    leds.setColor((byte)255, (byte)255, (byte)255);
  }

  private void configureBindings() {

    //============
    //Driver Binds
    //============
    driveTrain.setDefaultCommand(new DefaultDrive(driveTrain, new AxisSupplier(stick.getHID(), 1, 2.0, 0.01, true), 
                                 new AxisSupplier(stick.getHID(), 0, 2.0, 0.01, true),
                                 () -> {return drivingReversed;}
      )
    );

    //swap modes

    //Drop into trough
    stick.button(2).whileTrue(new InstantCommand(() -> coralManipulator.setIntake(CoralManipulatorConstants.DEFAULT_INTAKE_OUT_SPEED, CoralManipulatorConstants.DEFAULT_INTAKE_OUT_SPEED + 0.1))
                                                    .repeatedly().finallyDo(() -> coralManipulator.setIntake(0)));

    
    //drop game pieces with trigger
    stick.button(1).whileTrue(
      new InstantCommand(() -> coralManipulator.setIntake(CoralManipulatorConstants.DEFAULT_INTAKE_OUT_SPEED), coralManipulator).repeatedly()
      .finallyDo(() -> coralManipulator.setIntake(0)));

    //Move Front Camera
    stick.povUp().onTrue(new InstantCommand(() -> vision.setFrontCameraServo(VisionSubsystemConstants.HIGH_CAMERA_ANGLE)));
    stick.povDown().onTrue(new InstantCommand(() -> vision.setFrontCameraServo(VisionSubsystemConstants.LOW_CAMERA_ANGLE)));

    //Automatic Target Tracking 
    //Left Offset
    stick.button(3).whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(() -> vision.setFrontCameraServo(VisionSubsystemConstants.LOW_CAMERA_ANGLE)),
        new SteerToTarget(
          new AxisSupplier(stick::getY, 1.5, 0.01, true),
          () -> {return drivingReversed;}, 
          VisionSubsystemConstants.REEF_OFFSET_LEFT_M,
          () -> vision.frontCamGetYDistToTarget(VisionSubsystemConstants.REEF_OFFSET_LEFT_M), 
          1.2, 0.0005, 
          driveTrain
        )
      )
    );

    //Right Offset
    stick.button(4).whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(() -> vision.setFrontCameraServo(VisionSubsystemConstants.LOW_CAMERA_ANGLE)),
        new SteerToTarget(
          new AxisSupplier(stick::getY, 1.5, 0.01, true),
          () -> {return drivingReversed;}, 
          VisionSubsystemConstants.REEF_OFFSET_RIGHT_M,
          () -> vision.frontCamGetYDistToTarget(VisionSubsystemConstants.REEF_OFFSET_RIGHT_M), 
          1.2, 0.0005, 
          driveTrain
        )
      )
    );
    



    //=================
    //Manipulator Binds
    //=================
    coralManipulator.setDefaultCommand(
      new CoralManipulatorDefaultCommand(new AxisSupplier(controller::getLeftY, 1, 0, true),
                                         controller.povRight(), 
                                         controller.povLeft(),
                                         coralManipulator
      )
    );

    //Drop into trough by running intake wheels at different speeds
    controller.povUp().whileTrue(new InstantCommand(() -> coralManipulator.setIntake(CoralManipulatorConstants.DEFAULT_INTAKE_OUT_SPEED, CoralManipulatorConstants.DEFAULT_INTAKE_OUT_SPEED + 0.1))
                                                    .repeatedly().finallyDo(() -> coralManipulator.setIntake(0)));

    elevator.setDefaultCommand(new ElevatorDefaultCommand(new AxisSupplier(controller::getRightY, 1, 0, true), elevator));

    //Elevator and Coral Manipulator Setpoints
    //Home
    controller.a().onTrue(CoralandElevatorSetpointSequence.runSequence(CoralManipulatorConstants.SETPOINT_HOME_DEG, ElevatorConstants.SETPOINT_HOME, coralManipulator, elevator));
    //Trough 
    controller.x().onTrue(CoralandElevatorSetpointSequence.runSequence(CoralManipulatorConstants.SETPOINT_TROUGH_DEG, ElevatorConstants.SETPOINT_TROUGH, coralManipulator, elevator));
    //L2
    controller.y().onTrue(CoralandElevatorSetpointSequence.runSequence(CoralManipulatorConstants.SETPOINT_REEF_DEG, ElevatorConstants.SETPOINT_L2, coralManipulator, elevator));
    //L3
    controller.b().onTrue(CoralandElevatorSetpointSequence.runSequence(CoralManipulatorConstants.SETPOINT_REEF_DEG, ElevatorConstants.SETPOINT_L3, coralManipulator, elevator));
    //Coral Station
    controller.rightBumper().onTrue(CoralandElevatorSetpointSequence.runSequence(CoralManipulatorConstants.SETPOINT_STATION_DEG, ElevatorConstants.SETPOINT_STATION, coralManipulator, elevator));

    //toggle coral manipulator control
    controller.back().onTrue(new InstantCommand(() -> {
      if (coralManipulator.getState() == AngleControlState.DISABLED) {
        coralManipulator.enable();
      }
      else {
        coralManipulator.disable();
      }
    }));

    //automated intake cycles
    SequentialCommandGroup coralAutoIntakeCycle = 
    new SequentialCommandGroup(
      new SetCoralManipulatorAndElevator(CoralManipulatorConstants.SETPOINT_STATION_DEG, ElevatorConstants.SETPOINT_STATION, coralManipulator, elevator),
      new InstantCommand(() -> coralManipulator.setIntake(CoralManipulatorConstants.DEFAULT_INTAKE_IN_SPEED))
                        .repeatedly()
                        .until(coralManipulator::isLoaded)
                        .finallyDo(() -> coralManipulator.setIntake(0)),
      new InstantCommand(
        () -> {
          controller.setRumble(RumbleType.kBothRumble, 1);
          leds.setEffect(LEDEffect.BLINK);
        }
      )
    );

    //Bind automatic intake cycles
    controller.rightTrigger().whileTrue(coralAutoIntakeCycle);

    //home elevator
    controller.start().onTrue(new InstantCommand(elevator::autoHome, elevator).until(() -> {return elevator.getState() != Elevator.ElevatorState.HOMING;}));

    //prepare kicker
    new Trigger(() -> controller.getLeftTriggerAxis() > 0.1).onTrue(new SetAlgaeRemover(AlgaeRemoverConstants.SETPOINT_READY_DEG, algaeRemover));
    //activate kicker
    new Trigger(() -> controller.getLeftTriggerAxis() > 0.98).onTrue(new SetAlgaeRemover(AlgaeRemoverConstants.SETPOINT_ACTIVE_DEG, algaeRemover));
    //deactivate kicker
    new Trigger(() -> controller.getLeftTriggerAxis() > 0.98).onFalse(new SetAlgaeRemover(AlgaeRemoverConstants.SETPOINT_READY_DEG, algaeRemover));
    //home kicker
    controller.leftBumper().onTrue(new SetAlgaeRemover(AlgaeRemoverConstants.SETPOINT_HOME_DEG, algaeRemover));

    //stop execution of setpoint commands and return all mechanisms to manual control
    controller.leftStick().onTrue(
      new InstantCommand(
        () -> {
          CommandScheduler.getInstance().cancelAll();
        }
      )
    );




    
    //==================
    //Sensor state binds
    //==================
    /*new Trigger(coralManipulator::isLoaded).onFalse(new InstantCommand(
      () -> {
        controller.setRumble(RumbleType.kBothRumble, 0);
        leds.setEffect(LEDEffect.SOLID);
      }
    ));*/
    
  }

  private void configureAutoCommands() {
    
    //Register named commands here

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Selector", autoChooser);
  }

  public void onEnable() {
    algaeRemover.enable();
    coralManipulator.enable();
    elevator.enable();
  }

  public void onDisable() {
    algaeRemover.enable();
    coralManipulator.disable();
    elevator.disable();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
