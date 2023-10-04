// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Tools.TriggerButton;

/** Add your docs here. */
public class OI {
    public static XboxController driverController = new XboxController(0);
    public static JoystickButton buttonA = new JoystickButton(driverController, 1);
    public static JoystickButton buttonB = new JoystickButton(driverController, 2);
    public static JoystickButton buttonX = new JoystickButton(driverController, 3);
    public static JoystickButton buttonY = new JoystickButton(driverController, 4);
    public static JoystickButton rBumper = new JoystickButton(driverController, 6);
    public static JoystickButton lBumper = new JoystickButton(driverController, 5);
    public static BooleanSupplier rtSupplier = () -> getRTPercent() > Constants.OperatorConstants.RIGHT_TRIGGER_DEADZONE;
    public static BooleanSupplier ltSupplier = () -> getLTPercent() > Constants.OperatorConstants.LEFT_TRIGGER_DEADZONE;
    public static TriggerButton rt = new TriggerButton(rtSupplier);
    public static TriggerButton lt = new TriggerButton(ltSupplier);
    public static JoystickButton menuButton = new JoystickButton(driverController, 8);
    public static JoystickButton viewButton = new JoystickButton(driverController, 7);


    public static double getRTPercent() {
        return driverController.getRightTriggerAxis();
    }

    public static double getLTPercent() {
        return driverController.getLeftTriggerAxis();
    }
    
    
    public static double getDriverLeftY(){
        if (Math.abs(driverController.getLeftY()) < 0.075){
            return 0;
        } else {
            return driverController.getLeftY();
        }
    }
    public static double getDriverLeftX(){
        if (Math.abs(driverController.getLeftX()) < 0.075){
            return 0;
        } else {
            return driverController.getLeftX();
        }
    }
    public static double getDriverRightY(){
        if (Math.abs(driverController.getRightY()) < 0.075){
            return 0;
        } else {
            return driverController.getRightY();
        }
    }
    public static double getDriverRightX(){
        if (Math.abs(driverController.getRightX()) < 0.075){
            return 0;
        } else {
            return driverController.getRightX();
        }
    }
}
