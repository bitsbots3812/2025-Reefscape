// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionSubsystemConstants;

public class VisionSubsystem extends SubsystemBase {

  Servo frontCameraServo = new Servo(VisionSubsystemConstants.FRONT_CAMERA_SERVO_PORT);

  double currentServoPosition;

  public enum VisionPipeline {
    APRILTAG(0),
    OBJECT(1);

    int id;

    VisionPipeline(int id) {
      this.id = id;
    }
  }

  private PhotonCamera frontCam = new PhotonCamera("frontCam");
  private PhotonCamera rearCam = new PhotonCamera("rearCam");

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    setFrontCameraServo(VisionSubsystemConstants.LOW_CAMERA_ANGLE);
  }

  public void setRearCamPipeline(VisionPipeline pipeline) {
    rearCam.setPipelineIndex(pipeline.id);
  }

  public double rearCamGetAngleToTarget(double defaultValue) {
    PhotonPipelineResult result = rearCam.getLatestResult();
    
    if (result.hasTargets()) {
      return result.getBestTarget().yaw;
    }
    else {
      return defaultValue;
    }
  }

  public double frontCamGetAngleToTarget(double defaultValue) {
    PhotonPipelineResult result = rearCam.getLatestResult();

    if (result.hasTargets()) {
      return result.getBestTarget().yaw;
    }
    else {
      return defaultValue;
    }
  }

  public Transform3d frontCamGetTransformToTarget() {
    PhotonPipelineResult result = frontCam.getLatestResult();

    if (result.hasTargets()) {
      return result.getBestTarget().getBestCameraToTarget();
    }
    else {
      return null;
    }

  }

  public void setFrontCameraServo(double setpoint) {

    currentServoPosition = setpoint;

    frontCameraServo.set(setpoint / 180.0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
