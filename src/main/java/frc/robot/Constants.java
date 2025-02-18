package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public class DrivetrainConstants {

        public static final int FL_MOTOR_ID = 10;
        public static final int FR_MOTOR_ID = 11;
        public static final int RL_MOTOR_ID = 12;
        public static final int RR_MOTOR_ID = 13;

        public static final boolean FL_INVERT = false;
        public static final boolean FR_INVERT = false;
        public static final boolean RL_INVERT = false;
        public static final boolean RR_INVERT = false;
    }

    public static final class ElevatorConstants {
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
    }
}
