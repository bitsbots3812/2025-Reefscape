package frc.robot;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.util.Units;
import frc.robot.common.ArmController.AngleUnit;
import frc.robot.common.ConstraintClasses.RangeConstraint;
import frc.robot.subsystems.PDP;

public class Constants {
    public static final class DrivetrainConstants {

        public static final int FL_MOTOR_ID = 10;
        public static final int FR_MOTOR_ID = 11;
        public static final int RL_MOTOR_ID = 12;
        public static final int RR_MOTOR_ID = 13;

        public static final boolean FL_INVERT = false;
        public static final boolean FR_INVERT = false;
        public static final boolean RL_INVERT = false;
        public static final boolean RR_INVERT = false;

        public static final double MOTOR_MAX_RPM = 5700.0;
        public static final double GEARBOX_RATIO = 10.71;
        public static final double WHEEL_SIZE_M = Units.inchesToMeters(6.0);
        public static final double TRACKWIDTH_M = Units.inchesToMeters(21.5);

        //Motor revolutions to robot distance traveled
        public static final double ENCODER_DISTANCE_CONVERSION_FACTOR = (WHEEL_SIZE_M * Math.PI) / GEARBOX_RATIO;
        //Motor RPM to robot speed in M/S
        public static final double ENCODER_VELOCITY_CONVERSION_FACTOR = (1.0 / 60.0) * ENCODER_DISTANCE_CONVERSION_FACTOR;

        public static final double MAX_LINEAR_VELOCITY_MS = ENCODER_VELOCITY_CONVERSION_FACTOR * MOTOR_MAX_RPM;
    }

    /*public static final class ElevatorConstants {

        //Distance Limits
        public static final double ELEVATOR_MAX_HEIGHT_M = Units.inchesToMeters(28);
        public static final double ELEVATOR_MIN_HEIGHT_M = 0;
        
        //Ports
        public static final int LEFT_ELEVATOR_MOTOR_ID = 99;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 98;

        public static final boolean LEFT_INVERT = false;
        public static final boolean RIGHT_INVERT = true;

        public static final int ENCODER_CHANNEL_A = 0;
        public static final int ENCODER_CHANNAL_B = 1;
    
        //PID Controller Constants
        public static final double PID_P = 0.00004;
        public static final double PID_I = 0.0;
        public static final double PID_D = 0.0;
    
        //FeedForward Controller Constants
    
        //System Constants
        public static final double ELEVATOR_ALLOWED_ACCEL_MSS = 1.0;
        public static final double MOTOR_GEAR_RATIO = 26.0;
        public static final double PINION_GEAR_DIAMETER_M = Units.inchesToMeters(3);
        public static final double ELEVATOR_MOTOR_RPS_MAX = 5700.0/60.0;
        public static final double ELEVATOR_MAX_SPEED_MS = (ELEVATOR_MOTOR_RPS_MAX / MOTOR_GEAR_RATIO) * PINION_GEAR_DIAMETER_M * Math.PI;
        //Converts the motor revolutions into meters of elevator travel:
        public static final double DISTANCE_PER_PULSE_M = (PINION_GEAR_DIAMETER_M * Math.PI) / 8192.0;
    }*/

    public static final class ElevatorConstants {

        //Ports
        public static final int LEFT_ELEVATOR_MOTOR_ID = 99;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 98;

        public static final boolean LEFT_INVERT = false;
        public static final boolean RIGHT_INVERT = true;

        public static final int ENCODER_CHANNEL_A = 0;
        public static final int ENCODER_CHANNAL_B = 1;
    
        //Current Limits
        public static final double STALL_THRESHOLD_AMPS = 40.0; //TODO: tune this
        public static final int ELEVATOR_MOTOR_SMART_CURRENT_LIMIT = 40;
    
        public static final double VELOCITY_TOLERANCE_MS = 0.001;

        //Autohome constants
        public static final double AUTOHOME_OUTPUT_PERCENT = 0.1;
        public static final double AUTOHOME_WAIT_TIME_SEC = 0.5;
    
        //Setpoints
        public static final double SETPOINT_L2 = Units.inchesToMeters(15);
        public static final double SETPOINT_L3 = Units.inchesToMeters(26);
        public static final double SETPOINT_STATION = Units.inchesToMeters(10);
        public static final double SETPOINT_HOME = 0;
    
        //PID Controller Constants
        public static final double PID_P = 5;
        public static final double PID_I = 0.0;
        public static final double PID_D = 0.0;

        //Feed forward controller constatns
        public static final double FF_KG = 0.1;
    
        public static final double SETPOINT_TOLERANCE = 0.01; //1 cm

        //Manual Control Rate
        public static final double MANUAL_CONTROL_RATE_METER_SEC = 0.5;
    
        //FeedForward Controller Constants
    
        //System Constants
        public static final double ELEVATOR_MAX_HEIGHT_M = Units.inchesToMeters(20);
        
        public static final double ELEVATOR_ALLOWED_ACCEL_MSS = 1.0;
        public static final double MOTOR_GEAR_RATIO = 26.0;
        public static final double PINION_GEAR_DIAMETER_M = Units.inchesToMeters(3);
        public static final double ELEVATOR_MOTOR_RPS_MAX = 5700.0/60.0;
        public static final double ELEVATOR_MAX_SPEED_MS = (ELEVATOR_MOTOR_RPS_MAX / MOTOR_GEAR_RATIO) * PINION_GEAR_DIAMETER_M * Math.PI;
        //Converts the motor revolutions into meters of elevator travel:
        public static final double DISTANCE_PER_PULSE_M = (PINION_GEAR_DIAMETER_M * Math.PI) / 8192.0;
      }
    

    public static final class AlgaeManipulatorConstants {
        //Ports
        public static final int LEFT_TILT_MOTOR_ID = 2364839; //Numbers below are placeholders
        public static final int RIGHT_TILT_MOTOR_ID = 2364839;
        public static final int CENTRAL_INTAKE_MOTOR_ID = 2364839;

        public static final int LEFT_ENCODER_CHANNEL = 0;
        public static final int RIGHT_ENCODER_CHANNEL = 0;

        public static final int DIGITALINPUT_ID = 2364839;

        //Inversions
        public static final boolean LEFT_TILT_MOTOR_INVERTED = false;
        public static final boolean RIGHT_TILT_MOTOR_INVERTED = true;
        public static final boolean LEFT_TILT_ENCODER_INVERTED = false;
        public static final boolean RIGHT_TILT_ENCODER_INVERTED = false;
        public static final boolean INTAKE_MOTOR_INVERTED = false;

        //PID Controller Constants
        public static final double PID_P = 0.01;
        public static final double PID_I = 0;
        public static final double PID_D = 0;

        //Angle Information
        public static final double ABSOLUTE_ENCODER_OFFSET = 0;
        public static final double ANGLE_SETPOINT_TOLERANCE = 0;
        public static final RangeConstraint allowedAngleRange = new RangeConstraint(0, 90);

        //Setpoints
        public static final double SETPOINT_HOME_DEG = 90;
        public static final double SETPOINT_INTAKE_DEG = 45;
        public static final double SETPOINT_CLIMB_DEG = 3;

        //Feedforward Gains
        public static final double FF_KS = 0;
        public static final double FF_KG = 0;
        public static final double FF_KV = 0;

        //Motion profile constraints
        public static final double MAX_ANGULAR_VELOCITY = 0;
        public static final double MAX_PROFILED_ANGULAR_ACCELERATION = 0;

        //Angle Unit
        public static final AngleUnit angleUnit = AngleUnit.DEGREES;

        //Manual Control Rate
        public static final double MANUAL_CONTROL_RATE_DEG_SEC = 40.0;
        public static final double DEFAULT_INTAKE_OUT_SPEED = 0.5;
        public static final double DEFAULT_INTAKE_IN_SPEED = -0.5;
    }

    public static final class CoralManipulatorConstants {
        //Ports
        public static final int TILT_MOTOR_ID = 2025; //any values are placeholders FYI
        public static final int INTAKE_MOTOR_ID_LEFT = 2025;
        public static final int INTAKE_MOTOR_ID_RIGHT = 2025;

        public static final int ENCODER_CHANNEL = 2025;

        //inversions
        public static final boolean TILT_MOTOR_INVERTED = false;
        public static final boolean INTAKE_MOTOR_LEFT_INVERTED = false;
        public static final boolean INTAKE_MOTOR_RIGHT_INVERTED = false;
        public static final boolean ENCODER_INVERTED = false;

        //PID contoller consts
        public static final int PID_P = 0;
        public static final int PID_I = 0;
        public static final int PID_D = 0;
        //Angle Information
        public static final double ABSOLUTE_ENCODER_OFFSET = 0;
        public static final double ANGLE_SETPOINT_TOLERANCE = 0;
        public static final RangeConstraint allowedAngleRange = new RangeConstraint(-10, 90);

        //Setpoints
        public static final double SETPOINT_TROUGH_DEG = -10.0;
        public static final double SETPOINT_REEF_DEG = 0.0;
        public static final double SETPOINT_HOME_DEG = 90.0;
        public static final double SETPOINT_STATION_DEG = 30.0;

        //FeedForward gains
        public static final double FF_KS = 0;
        public static final double FF_KG = 0;
        public static final double FF_KV = 0;
        //Motion profile constraints
        public static final double MAX_ANGULAR_VELOCITY = 0;
        public static final double MAX_PROFILED_ANGULAR_ACCELERATION = 0;
        //Color sensor 
        public static final int redVal = 255;
        public static final int greenVal = 255;
        public static final int blueVal = 255;

        public static final int SENSOR_PROXIMITY_THRESHOLD = 500;

        //Angle Unit
        public static final AngleUnit angleUnit = AngleUnit.DEGREES;
        //Manual Control Rate
        public static final double MANUAL_CONTROL_RATE_DEG_SEC = 40.0;
        public static final double DEFAULT_INTAKE_OUT_SPEED = 0.5;
        public static final double DEFAULT_INTAKE_IN_SPEED = -0.5;
    }

    public static final class ClimberConstants {
        public static final int MOTOR_ID = 99;
        public static final boolean MOTOR_INVERTED = false;
        public static final double DEFAULT_SPEED = 1.0;
        public static final int LIMIT_SWITCH_PORT = 99;
    }

    public static final class PDPConstants {

        public static final int LEFT_ELEVATOR_MOTOR_PDP_PORT = 0;
        public static final int RIGHT_ELEVATOR_MOTOR_PDP_PORT = 0;

    }
}
