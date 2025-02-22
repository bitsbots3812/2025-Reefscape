package frc.robot.common;

public class Util {
  
  /**
   * Maps one range of values to a new range. 
   * @param Value
   * @param FromMin
   * @param FromMax
   * @param ToMin
   * @param ToMax
   * @return
   */
  public static double map(double Value, double FromMin, double FromMax, double ToMin, double ToMax) {
    return (((Value - FromMin) / (FromMax - FromMin)) * (ToMax - ToMin)) + ToMin;
  }
  
}