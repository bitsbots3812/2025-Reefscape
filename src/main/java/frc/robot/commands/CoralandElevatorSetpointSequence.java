// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class CoralandElevatorSetpointSequence {

    private CoralandElevatorSetpointSequence() {}

    public static Command get(CoralManipulator coralManipulator, Elevator elevator, double coralManipulatorSetpoint, double elevatorSetpoint) {
        
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        if (coralManipulatorSetpoint > CoralManipulatorConstants.SAFTEY_THRESHOLD_DEG && !ElevatorConstants.SAFE_RANGE_M.inRange(elevatorSetpoint)) {
            return null;
        } 
        
        if ()
        

        




        return sequence;
        
    }

}
