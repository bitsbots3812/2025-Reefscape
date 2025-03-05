package frc.robot.common;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;

public class AxisSupplier implements DoubleSupplier {
    private GenericHID hidController;
    private int axisId;
    private DoubleSupplier rawAxis;
    private double nonlinearity;
    private double deadzone;
    private boolean inverted;

    //Creates class for an axis that can be passed to commands. Note: Deadzones are applied before squaring.
    public AxisSupplier(GenericHID HIDController, int AxisID, double Nonlinearity, double Deadzone, boolean Inverted) {
        rawAxis = null;
        hidController = HIDController;
        axisId = AxisID;
        nonlinearity = Nonlinearity;
        deadzone = Deadzone;
        inverted = Inverted;
    }

    public AxisSupplier(DoubleSupplier RawAxis, double Nonlinearity, double Deadzone, boolean Inverted) {
        hidController = null;
        axisId = 0;
        rawAxis = RawAxis;
        nonlinearity = Nonlinearity;
        deadzone = Deadzone;
        inverted = Inverted;
    }
    
    public double getAsDouble () {

        double value;

        if (rawAxis == null) {
            value = MathUtil.applyDeadband(hidController.getRawAxis(axisId), deadzone);
        }
        else {
            value = MathUtil.applyDeadband(rawAxis.getAsDouble(), deadzone);
        }

        return (inverted ? -1 : 1) * Math.copySign(Math.pow(value, nonlinearity), value);
    }
}