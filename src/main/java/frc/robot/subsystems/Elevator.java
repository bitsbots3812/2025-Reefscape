// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

  //TODO:
  /*
   * Limit Switches
   *  Automatically Zero When Limit Switch is Detected
   * Auto-Home
   * Enable/Disable Control
   * Stall Detection
   * Soft Distance Limits
   *  Allow Movement only away from limit when distance limit is hit
   * 
   */

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

    leftElevatorMotor.setInverted(ElevatorConstants.LEFT_INVERT);
    rightElevatorMotor.setInverted(ElevatorConstants.RIGHT_INVERT);

    leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
    rightElevatorMotor.setNeutralMode(NeutralMode.Brake);

    rightElevatorMotor.follow(leftElevatorMotor);

    elevatorRelativeEncoder.setDistancePerPulse(ElevatorConstants.DISTANCE_PER_PULSE_M);
    elevatorRelativeEncoder.reset();
  }

  public double getPosition() {
    return elevatorRelativeEncoder.getDistance();
  }

  public double getVelocity() {
    return elevatorRelativeEncoder.getRate();
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

  public void enableControl() {
    
  }

  public void disableControl() {

  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Encoder (elevator): ", getPosition());
    
    //if executing a profiled sequence, currentTargetSate will not be null:
    if (currentTargetState != null) {
      TrapezoidProfile.State target = motionProfile.calculate(motionProfileTimer.get(), currentInitialState, currentTargetState);
      leftElevatorMotor.set(ControlMode.PercentOutput, elevatorPID.calculate(getPosition(), target.position) + elevatorFF.calculate(target.velocity));

      if (MathUtil.isNear(0, target.velocity, 0.05)) {
        setPoint = currentTargetState.position;
        currentTargetState = null;
      }
    }
    else {
      leftElevatorMotor.set(ControlMode.PercentOutput, elevatorPID.calculate(getPosition(), setPoint));
    }
  }
}