// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

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

    public static final double PLACEMENT_LOCATION_X_RED = 14.75;
    public static final double PLACEMENT_LOCATION_X_BLUE = FIELD_LENGTH - PLACEMENT_LOCATION_X_RED;
    public static final double[] PLACEMENT_LOCATIONS_Y_RED = new double[] {0.52, 1.07, 1.62, 2.2, 2.74, 3.3, 3.85, 4.43, 4.98};
    public static final double[] PLACEMENT_LOCATIONS_Y_BLUE = new double[] {4.98, 4.43, 3.85, 3.3, 2.74, 2.2, 1.07, 0.52};

    public static final double PLACEMENT_PATH_MIDPOINT_X_RED = 14.41;
    public static final double PLACEMENT_PATH_MIDPOINT_X_BLUE = FIELD_LENGTH - PLACEMENT_PATH_MIDPOINT_X_RED;
    public static final double[] PLACEMENT_PATH_MIDPOINT_Y_RED = new double[] {0.52, 1.07, 1.62, 2.2, 2.74, 3.3, 3.85, 4.43, 4.98};
    public static final double[] PLACEMENT_PATH_MIDPOINT_Y_BLUE = new double[] {4.98, 4.43, 3.85, 3.3, 2.74, 2.2, 1.07, 0.52};


    public static final double ROBOT_RADIUS = inchesToMeters(16); //m

    public static final double WHEEL_DIAMETER = inchesToMeters(4); //m

    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; // m

    public static final double TOP_SPEED = feetToMeters(16.3); // m/sec
    // public static final double TOP_SPEED = feetToMeters(8); // m/sec

    // public static final double TOP_SPEED = 3; // m/sec

    // used to be 6.75 for SDS mk4, 6.37 for Morris
    public static final double GEAR_RATIO = 6.75;

    // Morris 2.0 is 12.8
    public static final double STEER_GEAR_RATIO = 12.8;

    public static final double VERTICAL_CLIMBER_GEAR_RATIO = 10.0;

    public static final double ROTATING_CLIMBER_GEAR_RATIO = 115.0;

    public static final double CLIMBER_INCHES_PER_ROTATION = 4.0;

    public static final double FALCON_TICS_PER_ROTATION = 2048;

    public static final double NEO_TICS_PER_ROTATION = 42;

    public static final double ROBOT_WIDTH = inchesToMeters(29); //m

    public static final double MODULE_OFFSET = inchesToMeters(3);

    public static final double lookUpTableRPMOffset = -50; // changed from 0;

    public static final double lookUpTableHoodOffset = 0.5; // changed from 0;


    public static double getClimberFalconTics(double inches) {
        return (inches / CLIMBER_INCHES_PER_ROTATION) * VERTICAL_CLIMBER_GEAR_RATIO * FALCON_TICS_PER_ROTATION;
    }

    public static double getNeoTics(double degrees) {
        return ((degrees / 360.0)) * (ROTATING_CLIMBER_GEAR_RATIO);
    }

    public static double getVerticalClimberInches(double tics) {
        return ((tics / FALCON_TICS_PER_ROTATION) / VERTICAL_CLIMBER_GEAR_RATIO) * CLIMBER_INCHES_PER_ROTATION;
    }

    public static double getRotatingClimberAngle(double tics) {
        return ((tics) / ROTATING_CLIMBER_GEAR_RATIO) * 360.0;
    }

    //{distance(inches), hood angle, RPM}
    public static final double[][] SHOOTING_LOOKUP_TABLE = {
        // {70.0, 7.5, 1350.0},
        // {79.0, 8.5, 1420.0},
        // {85.0, 8.5, 1400.0},
        // {107.0, 13.25, 1420.0},
        // {112.0, 15.0, 1420.0},
        // {124.0, 16.0, 1420.0},
        // {133.0, 17.0, 1460.0},
        // {139.0, 18.0, 1480.0},
        // {143.0, 18.0, 1500.0},
        // {154.0, 19.0, 1520.0},
        // {159.0, 20.0, 1530.0},
        // {164.0, 22.5, 1530.0},
        // {170.0, 24.0, 1560.0},
        // {180.0, 25.0, 1580.0},
        // {190.0, 27.5, 1650.0}
        {70.0, 10.0, 1400.0},
        {90.0, 10.0, 1430.0},
        {110.0, 15.25, 1460.0},
        {130.0, 19, 1510.0},
        {150.0, 21, 1560.0},
        {165.0, 23.0, 1600},
        {170.0, 24.25, 1620},
        {190.0, 25.5, 1700},
        {210.0, 27.0, 1720},
        {230.0, 32, 1780}
    };
    
    public static double[] getShooterValues(double distance) {
        int lastIndex = SHOOTING_LOOKUP_TABLE.length - 1;
        if (distance < SHOOTING_LOOKUP_TABLE[0][0]) {
            //If the distance is closer than the first setpoint
            double[] returnArr = {SHOOTING_LOOKUP_TABLE[0][1] + lookUpTableHoodOffset, lookUpTableRPMOffset + SHOOTING_LOOKUP_TABLE[0][2]};
            return returnArr;
        } else if (distance > SHOOTING_LOOKUP_TABLE[lastIndex][0]) {
            //If the distance is farther than the last setpoint
            double[] returnArr = {SHOOTING_LOOKUP_TABLE[lastIndex][1] + lookUpTableHoodOffset, lookUpTableRPMOffset + SHOOTING_LOOKUP_TABLE[lastIndex][2]};
            return returnArr;
        } else {
            for (int i = 0; i < SHOOTING_LOOKUP_TABLE.length; i ++) {
                if (distance > SHOOTING_LOOKUP_TABLE[i][0] && distance < SHOOTING_LOOKUP_TABLE[i + 1][0]) {
                    //If the distance is in the table of setpoints
                    //Calculate where distance is between setpoints
                    double leftDif = distance - SHOOTING_LOOKUP_TABLE[i][0];
                    double percent = leftDif / (SHOOTING_LOOKUP_TABLE[i + 1][0] - SHOOTING_LOOKUP_TABLE[i][0]);

                    double hood1 = SHOOTING_LOOKUP_TABLE[i][1];
                    double rpm1 = SHOOTING_LOOKUP_TABLE[i][2];
                    double hood2 = SHOOTING_LOOKUP_TABLE[i + 1][1];
                    double rpm2 = SHOOTING_LOOKUP_TABLE[i + 1][2];

                    //Interpolate in-between values for hood angle and shooter rpm
                    double newHood = hood1 + lookUpTableHoodOffset + (percent * (hood2 - hood1));
                    double newRPM = lookUpTableRPMOffset + rpm1 + (percent * (rpm2 - rpm1));
                    
                    double[] returnArr = {newHood, newRPM};
                    return returnArr;
                }
            }
            //Should never run
            double[] returnArr = {0, 0};
            return returnArr;
        }  
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double feetToMeters(double feet) {
        double inches = feet * 12;
        return inchesToMeters(inches);
    }

    public static int shooterRPMToUnitsPer100MS(double rpm) {
        return (int) (Math.round((rpm / 600.0) * FALCON_TICS_PER_ROTATION));
    }

    public static double unitsPer100MsToRPM(double units) {
        return (units * 600) / (Constants.FALCON_TICS_PER_ROTATION);


    }

}
