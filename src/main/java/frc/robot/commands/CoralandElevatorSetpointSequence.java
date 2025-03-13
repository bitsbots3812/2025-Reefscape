// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.CoralManipulator.SetCoralManipulator;
import frc.robot.commands.Elevator.SetElevator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class CoralandElevatorSetpointSequence {

    private CoralandElevatorSetpointSequence() {}

    private static Command get(double coralManipulatorSetpoint, double elevatorSetpoint, CoralManipulator coralManipulator, Elevator elevator) {
        
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        //quit if trying to go to an invalid setpoint
        if (coralManipulatorSetpoint > CoralManipulatorConstants.SAFTEY_THRESHOLD_DEG && elevatorSetpoint != 0) {
            return null;
        } 

        //stop the elevator and coral manipulator wherever they are.
        coralManipulator.setPoint(coralManipulator.getAngle());
        elevator.forceSetPosition(elevator.getPosition());

        //Handle cases where the mechanism has been forced to an invalid setpoint
        if (elevator.getSetpoint() >= (ElevatorConstants.CROSSMEMBER_HEIGHT_M) && coralManipulator.getSetpoint() > CoralManipulatorConstants.SAFTEY_THRESHOLD_DEG) {
            sequence.addCommands(new SetCoralManipulator(CoralManipulatorConstants.SAFTEY_THRESHOLD_DEG, coralManipulator));
        }
        else if ((elevator.getSetpoint() != 0) && coralManipulator.getSetpoint() > CoralManipulatorConstants.SAFTEY_THRESHOLD_DEG) {
            sequence.addCommands(new SetElevator(0, elevator));
        }

        if (coralManipulatorSetpoint > CoralManipulatorConstants.SAFTEY_THRESHOLD_DEG) {
            if (elevator.getSetpoint() > 0) {
                sequence.addCommands(
                    new ParallelCommandGroup(
                        new SequentialCommandGroup (
                            new SetCoralManipulator(CoralManipulatorConstants.SAFTEY_THRESHOLD_DEG, coralManipulator),
                            Commands.waitUntil(() -> elevator.getPosition() < ElevatorConstants.SAFE_RANGE_M.getLowerConstraint()),
                            new SetCoralManipulator(coralManipulatorSetpoint, coralManipulator)
                        ),
                        new SetElevator(elevatorSetpoint, elevator)
                    )
                );
            }
            else {
                sequence.addCommands(new SetCoralManipulator(coralManipulatorSetpoint, coralManipulator));
            }
        }
        else {
            if (coralManipulator.getSetpoint() > CoralManipulatorConstants.SAFTEY_THRESHOLD_DEG) {
                sequence.addCommands(new SetCoralManipulator(CoralManipulatorConstants.SAFTEY_THRESHOLD_DEG, coralManipulator));
            }

            sequence.addCommands(
                new ParallelCommandGroup(
                    new SetCoralManipulator(coralManipulatorSetpoint, coralManipulator),
                    new SetElevator(elevatorSetpoint, elevator)
                )
            );
        }

        return sequence;
        
    }

    public static Command runSequence (double coralManipulatorSetpoint, double elevatorSetpoint, CoralManipulator coralManipulator, Elevator elevator) {
        return new InstantCommand(
            () -> {
                CommandScheduler.getInstance().schedule(get(coralManipulatorSetpoint, elevatorSetpoint, coralManipulator, elevator));
            }
        );
    }

}
