// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

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
  public VisionSubsystem() {}

  public void setRearCamPipeline(VisionPipeline pipeline) {
    rearCam.setPipelineIndex(pipeline.id);
  }

  public double rearCamGetAngleToTarget() {
    PhotonPipelineResult result = rearCam.getLatestResult();
    
    if (result.hasTargets()) {
      return result.getBestTarget().yaw;
    }
    else {
      return 0;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
