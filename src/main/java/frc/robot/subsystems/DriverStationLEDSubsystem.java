// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverStationLEDSubsystem extends SubsystemBase {
  /** Creates a new DriverStationLEDSubsystem. */

  public enum LEDEffect {

    SOLID("solid"),
    BLINK("blink"),
    FAST_BLINK("fst_blink");


    String name;
    
    LEDEffect(String name) {
      this.name = name;
    }
  }

  private IntegerArrayPublisher color;
  private StringPublisher pattern;

  public DriverStationLEDSubsystem() {
    color = NetworkTableInstance.getDefault().getIntegerArrayTopic("color").publish();
    pattern = NetworkTableInstance.getDefault().getStringTopic("pattern").publish();
  }

  public void setColor(byte r, byte g, byte b) {
    color.set(new long[] {r, g, b});
  }

  public void setEffect(LEDEffect effect) {
    pattern.set(effect.name);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
