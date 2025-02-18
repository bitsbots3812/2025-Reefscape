// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private VictorSPX leftElevatorMotor  = new VictorSPX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID);
  private VictorSPX rightElevatorMotor = new VictorSPX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID);
  private Encoder elevatorRelativeEncoder  = new Encoder(ElevatorConstants.ENCODER_CHANNEL_A, ElevatorConstants.ENCODER_CHANNAL_B);
  private double setPoint = 0;

  private PIDController elevatorPID = new PIDController(
    ElevatorConstants.PID_P,
    ElevatorConstants.PID_I,
    ElevatorConstants.PID_D
  );

  private ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0, 0,0,0,0);

  private TrapezoidProfile motionProfile = new TrapezoidProfile(new Constraints(ElevatorConstants.ELEVATOR_MAX_SPEED_MS, ElevatorConstants.ELEVATOR_ALLOWED_ACCEL_MSS));
  private TrapezoidProfile.State currentInitialState = null;
  private TrapezoidProfile.State currentTargetState  = null;

  private Timer motionProfileTimer = new Timer();

  /** Creates a new ElevatorSubsystem. */
  public Elevator() {
    leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
    rightElevatorMotor.setNeutralMode(NeutralMode.Brake);

    rightElevatorMotor.follow(leftElevatorMotor);

    elevatorRelativeEncoder.setDistancePerPulse(ElevatorConstants.POSITION_CONVERSION_FACTOR);
  }

  public double getPosition() {
    return elevatorRelativeEncoder.getRate();
  }

  public double getVelocity() {
    return elevatorRelativeEncoder.getDistance();
  }

  public void setProfiled(double setPoint) {
    //Creates states for the trapezoid profile with the current state and a state at the target position with zero velocity.
    currentInitialState = new TrapezoidProfile.State(getPosition(), getVelocity());
    currentTargetState =  new TrapezoidProfile.State(setPoint, 0);
    motionProfileTimer.restart();
  }

  public void set(double setPoint) {
    currentTargetState = null; //set the target state to null, alterting periodic() to quit any profiled motion.
    this.setPoint = setPoint;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Encoder (elevator): ", getPosition());
    
    //if executing a profiled sequence, currentTargetSate will not be null:
    if (currentTargetState != null) {
      TrapezoidProfile.State target = motionProfile.calculate(motionProfileTimer.get(), currentInitialState, currentTargetState);
      elevator.set(elevatorPID.calculate(getPosition(), target.position) + elevatorFF.calculate(target.velocity));

      if (MathUtil.isNear(0, target.velocity, 0.05)) {
        setPoint = currentTargetState.position;
        currentTargetState = null;
      }
    }
    else {
      elevator.set(elevatorPID.calculate(getPosition(), setPoint));
    }
  }
}