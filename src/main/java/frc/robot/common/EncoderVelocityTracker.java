package frc.robot.common;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

public class EncoderVelocityTracker {
    private double currentVelocity = 0;
    private double velocityPreviousAngle;
    private Timer  velocityTimer = new Timer();

    private DoubleSupplier getRawAngle;

    public EncoderVelocityTracker(DoubleSupplier getRawAngle) {
        this.getRawAngle = getRawAngle;
    }

    public double getAngle() {
        return getRawAngle.getAsDouble();
    }

    public void update() {
        double currentTime = velocityTimer.get();

        if (currentTime != 0) {
            currentVelocity = (getAngle() - velocityPreviousAngle) / currentTime;
        }

        velocityTimer.restart();
        velocityPreviousAngle = getAngle();
    }

    public double getVelocity() {
        return currentVelocity;
    }
    
}