package frc.robot.common;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;

public class AxisSupplier implements DoubleSupplier {
    GenericHID hidController;
    int axisId;
    double nonlinearity;
    double deadzone;
    boolean inverted;

    //Creates class for an axis that can be passed to commands. Note: Deadzones are applied before squaring.
    public AxisSupplier(GenericHID HIDController, int AxisID, double Nonlinearity, double Deadzone, boolean Inverted) {
        hidController = HIDController;
        axisId = AxisID;
        nonlinearity = Nonlinearity;
        deadzone = Deadzone;
        inverted = Inverted;
    }
    
    public double getAsDouble () {
        double value = MathUtil.applyDeadband(hidController.getRawAxis(axisId), deadzone);
        return (inverted ? -1 : 1) * Math.copySign(Math.pow(value, nonlinearity), value);
    }
}