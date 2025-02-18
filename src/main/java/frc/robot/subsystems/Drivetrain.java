// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants;


public class Drivetrain extends SubsystemBase {

  private final SparkMax frontLeft = new SparkMax(Constants.DrivetrainConstants.FL_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
  private final SparkMax frontRight = new SparkMax(Constants.DrivetrainConstants.FR_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxConfig frontRightConfig = new SparkMaxConfig();
  private final SparkMax rearLeft = new SparkMax(Constants.DrivetrainConstants.RL_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxConfig rearLeftConfig = new SparkMaxConfig();
  private final SparkMax rearRight = new SparkMax(Constants.DrivetrainConstants.RR_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxConfig rearRightConfig = new SparkMaxConfig();

  private DifferentialDrive Drivetrain = new DifferentialDrive(frontLeft::set, frontRight::set);

  
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontLeftConfig.inverted(Constants.DrivetrainConstants.FL_INVERT);
    frontLeftConfig.idleMode(IdleMode.kBrake);
    frontLeftConfig.smartCurrentLimit(40);

    frontRightConfig.inverted(Constants.DrivetrainConstants.FR_INVERT);
    frontRightConfig.idleMode(IdleMode.kBrake);
    frontRightConfig.smartCurrentLimit(40);

    rearLeftConfig.inverted(Constants.DrivetrainConstants.RL_INVERT);
    rearLeftConfig.idleMode(IdleMode.kBrake);
    rearLeftConfig.smartCurrentLimit(40);
    rearLeftConfig.follow(frontLeft);

    rearRightConfig.inverted(Constants.DrivetrainConstants.RR_INVERT);
    rearRightConfig.idleMode(IdleMode.kBrake);
    rearRightConfig.smartCurrentLimit(40);
    rearRightConfig.follow(frontRight);

    frontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRight.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearLeft.configure(rearLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearRight.configure(rearRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  
  public void drive(double xSpeed, double zRotation) {
    Drivetrain.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
