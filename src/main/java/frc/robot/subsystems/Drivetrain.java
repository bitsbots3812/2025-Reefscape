// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;


public class Drivetrain extends SubsystemBase {

  RobotConfig config;

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final SparkMax frontLeft = new SparkMax(Constants.DrivetrainConstants.FL_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
  private final SparkMax frontRight = new SparkMax(Constants.DrivetrainConstants.FR_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxConfig frontRightConfig = new SparkMaxConfig();
  private final SparkMax rearLeft = new SparkMax(Constants.DrivetrainConstants.RL_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxConfig rearLeftConfig = new SparkMaxConfig();
  private final SparkMax rearRight = new SparkMax(Constants.DrivetrainConstants.RR_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxConfig rearRightConfig = new SparkMaxConfig();

  private RelativeEncoder leftEncoder  = frontLeft.getEncoder();
  private RelativeEncoder rightEncoder = frontRight.getEncoder();

  private DifferentialDrive           drivetrain = new DifferentialDrive(frontLeft::set, frontRight::set);
  private DifferentialDriveOdometry   odometry   = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DrivetrainConstants.TRACKWIDTH_M);

  
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontLeftConfig.inverted(Constants.DrivetrainConstants.FL_INVERT);
    frontLeftConfig.idleMode(IdleMode.kBrake);
    frontLeftConfig.smartCurrentLimit(40);
    frontLeftConfig.encoder.positionConversionFactor(DrivetrainConstants.ENCODER_DISTANCE_CONVERSION_FACTOR);
    frontLeftConfig.encoder.velocityConversionFactor(DrivetrainConstants.ENCODER_VELOCITY_CONVERSION_FACTOR);

    frontRightConfig.inverted(Constants.DrivetrainConstants.FR_INVERT);
    frontRightConfig.idleMode(IdleMode.kBrake);
    frontRightConfig.smartCurrentLimit(40);
    frontRightConfig.encoder.positionConversionFactor(DrivetrainConstants.ENCODER_DISTANCE_CONVERSION_FACTOR);
    frontRightConfig.encoder.velocityConversionFactor(DrivetrainConstants.ENCODER_VELOCITY_CONVERSION_FACTOR);

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


    
    //CONFIGURE PATHPLANNER
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
  
    AutoBuilder.configure(
      this::getPose,
      this::resetPose,
      this::getRobotRelativeSpeeds,
      (speeds, feedforwards) -> driveRobotRelative(speeds),
      new PPLTVController(0.02),
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
    
  }

  

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    gyro.reset();
    gyro.setAngleAdjustment(pose.getRotation().getDegrees());
    odometry.resetPose(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity()));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    
    drivetrain.tankDrive (
      wheelSpeeds.leftMetersPerSecond / DrivetrainConstants.MAX_LINEAR_VELOCITY_MS,
      wheelSpeeds.rightMetersPerSecond / DrivetrainConstants.MAX_LINEAR_VELOCITY_MS
    );
    
  }
  
  public void drive(double xSpeed, double zRotation) {
    drivetrain.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Gyro Rotation: ", gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Odometry X Distance (m): ", getPose().getX());
    SmartDashboard.putNumber("Odometry Y Distance (m): ", getPose().getY());

    //Update Odometry
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }


}
