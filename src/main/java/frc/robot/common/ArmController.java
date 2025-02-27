// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.ConstraintClasses.RangeConstraint;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ArmController {

    public enum AngleUnit {
        DEGREES(Math.PI / 180.0, "Degrees"),
        RADIANS(1, "Radians"),
        ROTATIONS(2 * Math.PI, "Rotations");

        double conversionFactorToRadians;
        String displayString;

        AngleUnit (double conversionFactorToRadians, String displayString) {
            this.conversionFactorToRadians = conversionFactorToRadians;
            this.displayString = displayString;
        }
    }

    public enum AngleControlState {
        POSITION_CONTROL("Position Control"),
        //VELOCITY_CONTROL("Velocity Control"),
        PROFILED_CONTROL("Profiled Control"),
        DISABLED("Disabled"),
        /*FAULT("Hardware Fault")*/;
    
        String displayName;
    
        AngleControlState(String displayName) {
          this.displayName = displayName;
        };
    }
    
    //=========================
    //      Constatnts
    //=========================
    private String displayName;

    private DoubleConsumer setAngleMotor;
    private DoubleSupplier getAbsoluteEncoderAngle;
    private DoubleSupplier getAngularVelocity;

    private final double ABSOLUTE_ENCODER_OFFSET;

    private RangeConstraint allowedAngleRange;

    private final double ANGLE_SETPOINT_TOLERANCE;

    //======================
    //      Variables
    //======================
    private ArmFeedforward ff;
    private PIDController pid;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State currentInitialState = null;
    private TrapezoidProfile.State currentTargetState  = null;
    private Timer motionProfileTimer = new Timer();

    private AngleControlState currentControlState = AngleControlState.DISABLED;
    private double setpoint = 0;

    boolean debugDataEnabled = true;

    private AngleUnit angleUnit;


    //Constructor
    /**
     * Creates an ArmController object. It is expected that an angle of 0 is parellel to the floor and positive angles go up.
     * 
     * |
     * | <-. (+)
     * |    \
     * +------------> 0
     *    <Floor>
     * 
     * @param setAngleMotor a doubleConsumer that sets the angle motors. A positive input must correspond to the motor moving in the positive angular direction.
     * @param getAbsoluteEncoderAngle a DoubleSupplier that provides the angle from an absolute encoder in the unit defined by the angleUnit parameter
     * @param getAngularVelocity a DoubleSupplier that provides the angular velocity of the arm in angular units per second
     * @param ABSOLUTE_ENCODER_OFFSET an offset that will be subtracted from the output of getAbsoluteEncoderAngle
     * @param ANGLE_SETPOINT_TOLERANCE the number of angular units the arm is allowed to deviate from the setpoint while still being considered at the setpoint
     * @param allowedAngleRange the angles through which the arm is permitted to move
     * @param PID_P PID controller proportional gain
     * @param PID_I PID controller integral gain
     * @param PID_D PID controller derivative gain
     * @param FF_KS Feed Forward controller static gain
     * @param FF_KG Feed Forward controller gravity gain
     * @param FF_KV Feed Forward controller velocity gain
     * @param MAX_ANGULAR_VELOCITY The maximum angular velocity the arm is allowed to travel at during profiled motion and can achieve. Angular units per second.
     * @param MAX_PROFILED_ANGULAR_ACCELERATION The maximum allowed acceleration of the arm in profiled motion. Angular units per second per second.
     * @param displayName The name of the arm for display in SmartDashboard output.
     * @param angleUnit The angle unit used by all functions in this controller.
     */
    public ArmController (
        //Functional Interfaces
        DoubleConsumer setAngleMotor,
        DoubleSupplier getAbsoluteEncoderAngle,
        DoubleSupplier getAngularVelocity,

        //Angle Information
        double ABSOLUTE_ENCODER_OFFSET,
        double ANGLE_SETPOINT_TOLERANCE,
        RangeConstraint allowedAngleRange,
        
        //PID Gains
        double PID_P,
        double PID_I,
        double PID_D,

        //Feedforward Gains
        double FF_KS,
        double FF_KG,
        double FF_KV,

        //Motion profile constraints
        double MAX_ANGULAR_VELOCITY,
        double MAX_PROFILED_ANGULAR_ACCELERATION,

        //Display Strings
        String displayName,

        //Angle Unit Selection
        AngleUnit angleUnit
    ) 
    {
        this.setAngleMotor      = setAngleMotor;
        this.getAbsoluteEncoderAngle = getAbsoluteEncoderAngle;
        this.getAngularVelocity      = getAngularVelocity;

        this.ABSOLUTE_ENCODER_OFFSET  = ABSOLUTE_ENCODER_OFFSET;
        this.ANGLE_SETPOINT_TOLERANCE = ANGLE_SETPOINT_TOLERANCE;
        this.allowedAngleRange        = allowedAngleRange;

        pid = new PIDController(PID_P, PID_I, PID_D);
        ff  = new ArmFeedforward(FF_KS, FF_KG, FF_KV);

        motionProfile = new TrapezoidProfile(new Constraints(MAX_ANGULAR_VELOCITY, MAX_PROFILED_ANGULAR_ACCELERATION));

        this.displayName = displayName;
        this.angleUnit  = angleUnit;
    }

    /**
     * @param enabled Enables or disables output of information to SmartDashboard.
     */
    public void enableDebugOutput(boolean enabled) {
        debugDataEnabled = enabled;
    }

    /**
     * Enables control of the arm
     */
    public void enable() {
        currentControlState = AngleControlState.POSITION_CONTROL;
        setpoint = getAngle(); //Change setpoint to the current position to prevent erratic movement.
        pid.reset();
    }

    /**
     * Disables control of the arm (sets motor to zero)
     */
    public void disable() {
        currentControlState = AngleControlState.DISABLED;
    }

    /**
     * Sets the arm to position control mode, and changes the angle setpoint.
     * @param setpoint The angle setpoint. Will be clamped to the range set in allowedAngleRange.
     */
    public void setAngle(double setpoint) {
        if (currentControlState != AngleControlState.DISABLED) {
            currentControlState = AngleControlState.POSITION_CONTROL;
        }
        this.setpoint = allowedAngleRange.clamp(setpoint);
        pid.reset();
    }

    /**
     * @return The current angle from the absolute encoder with ABSOLUTE_ENCODER_OFFSET subtracted.
     */
    public double getAngle() {
        return getAbsoluteEncoderAngle.getAsDouble() - ABSOLUTE_ENCODER_OFFSET;
    }

    /**
     * @return Gives the current angular velocity as reported by the encoder.
     */
    public double getVelocity() {
        return getAngularVelocity.getAsDouble();
    }

    /**
     * Uses trapezoid profiles to move the arm to the given target angle.
     * @param setpoint The angle setpoint. Will be clamped to the range set in allowedAngleRange.
     */
    public void setProfiled(double setpoint) {
        if (currentControlState == AngleControlState.DISABLED) {
            return;
        }
        
        currentControlState = AngleControlState.PROFILED_CONTROL;
        currentInitialState = new TrapezoidProfile.State(getAngle(), getVelocity());
        currentTargetState  = new TrapezoidProfile.State(allowedAngleRange.clamp(setpoint), 0);

        this.setpoint = allowedAngleRange.clamp(setpoint);

        motionProfileTimer.restart();
        pid.reset();

    }

    /**
     * @return Returns true if the arm is within tolerance of the angle setpoint.
     */
    public boolean atSetpoint() {
        return MathUtil.isNear(setpoint, getAngle(), ANGLE_SETPOINT_TOLERANCE);
    }

    /**
     * 
     * @return The Current Angle Setpoint
     */
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * @return Returns the current state of the arm controller.
     */
    public AngleControlState getState() {
        return currentControlState;
    }


    /**
     * Must be called in a periodic method to run the arm.
     */
    public void execute() {

        if (debugDataEnabled) {
            SmartDashboard.putString(displayName + " State: ", getState().displayName);
            SmartDashboard.putNumber(displayName + "  Current Angle (" + angleUnit.displayString + "): ", getAngle());
            SmartDashboard.putNumber(displayName + "  Angular Velocity (" + angleUnit.displayString + " per sec): ", getVelocity());
            SmartDashboard.putNumber(displayName + "  Angle Setpoint (" + angleUnit.displayString + "): ", setpoint);
            SmartDashboard.putBoolean(displayName + "  at Setpoint: ", atSetpoint());
            //SmartDashboard.putNumber(displayName + "  Angle Motor Output Power: ", );
        }

        double pidOutput = 0;
        double ffOutput = 0;

        //Primary control handling
        switch (currentControlState) {
          case DISABLED:
            setAngleMotor.accept(0);
            break;

          case POSITION_CONTROL:
            pidOutput = pid.calculate(getAngle(), setpoint);
            ffOutput =  ff.calculate(setpoint * angleUnit.conversionFactorToRadians, 0);
            setAngleMotor.accept(pidOutput + ffOutput);
            break;

          case PROFILED_CONTROL:
            TrapezoidProfile.State target = motionProfile.calculate(motionProfileTimer.get(), currentInitialState, currentTargetState);
            pidOutput = pid.calculate(getAngle(), allowedAngleRange.clamp(target.position));
            ffOutput = ff.calculate(target.position * angleUnit.conversionFactorToRadians, target.velocity * angleUnit.conversionFactorToRadians);
            setAngleMotor.accept(pidOutput + ffOutput);

            if (target.position == setpoint) {
              currentControlState = AngleControlState.POSITION_CONTROL;
            }

            break;

        }

        SmartDashboard.putNumber(displayName + "PID Output: ", pidOutput);
        SmartDashboard.putNumber(displayName + "FF Output: ", ffOutput);
    }
}