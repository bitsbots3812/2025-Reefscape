// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ArmController;

public class CoralManipulator extends SubsystemBase {

  public DutyCycleEncoder coralTilt = new DutyCycleEncoder(0);
  public VictorSPX coralTilter = new VictorSPX(0);
  
  private ArmController CoralController = new ArmController();
  /** Creates a new CoralManipulator. */
  public CoralManipulator() {



  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
