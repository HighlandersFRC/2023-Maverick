// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double RIGHT_TRIGGER_DEADZONE = 0.1;
    public static final double LEFT_TRIGGER_DEADZONE = 0.1;
  }
  
  public static final double ROBOT_LENGTH = inchesToMeters(29);

  public static final double ROBOT_WIDTH = inchesToMeters(25);

  public static final double SWERVE_MODULE_OFFSET = inchesToMeters(2.5);

  public static final double ROBOT_RADIUS = inchesToMeters(15.375);

  public static final double GEAR_RATIO = 6.75;

  public static final double STEER_GEAR_RATIO = 150/7;

  public static final double TOP_SPEED = feetToMeters(20);

  public static final double WHEEL_DIAMETER = inchesToMeters(4);

  public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;

  public static final double Wheel_Rotations_In_A_Meter = 1 / WHEEL_CIRCUMFRENCE;

  public static final double FIELD_WIDTH = 8.2;
  
  public static final double FIELD_LENGTH = 16.63;

  public static final double[][] TAG_LOCATIONS = new double[][] {
    {15.513558, 1.071626, 0.462788},
    {15.513558, 2.748026, 0.462788},
    {15.513558, 4.424426, 0.462788},
    {16.178784, 6.749796, 0.695452},
    {0.36195, 6.749796, 0.695452},
    {1.02743, 4.4224426, 0.462788},
    {1.02743, 2.748026, 0.462788},
    {1.02743, 1.071626, 0.462788},
  };

  public static double degreesToRotations(double degrees){
    return degrees / 360;
  }

  public static double rotationsToDegrees(double rotations){
    return rotations * 360;
  }

  public static double radiansToRotations(double radians){
    return radians / (2 * Math.PI);
  }

  public static double rotationsToRadians(double rotations){
    return rotations * (2 * Math.PI);
  }

  public static double inchesToMeters(double inches){
    return inches * 0.0254;
  }

  public static double metersToInches(double meters){
    return meters / 0.0254;
  }

  public static double feetToMeters(double feet){
    double inches = feet * 12;
    return inchesToMeters(inches);
  }

  public static double angleToUnitVectorI(double angle){
    return (Math.cos(angle));
  }

  public static double angleToUnitVectorJ(double angle){
    return (Math.sin(angle));
  }

  public double MPSToRPS(double mps){
    return (mps * Constants.Wheel_Rotations_In_A_Meter);
  }

  public double RPSToMPS(double rps){
    return (rps / Constants.Wheel_Rotations_In_A_Meter);
  }
}
