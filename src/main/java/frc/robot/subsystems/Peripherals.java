// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tools.Navx;

public class Peripherals extends SubsystemBase {
  /** Creates a new Peripherals. */
  public Peripherals() {}

  public final static AHRS ahrs = new AHRS(Port.kMXP);

  public final static Navx navx = new Navx(ahrs);

  public void init(){
    zeroNavx();
  }

  public double getNavxAngle() {
    return navx.currentAngle();
  }

  private double navxStartingRoll = 0;

  public double getNavxRollOffset() {
    if(navxStartingRoll == 0) {
      navxStartingRoll = getNavxRoll();
      if(navxStartingRoll == 0) {
        navxStartingRoll = 0.0001;
      }
      return navxStartingRoll;
    } else {
      double offset = getNavxRoll() - navxStartingRoll;
      return offset;
    }
  }

  public void zeroNavx(){
    navx.softResetYaw();
    navx.softResetAngle();
    navx.softResetPitch();
  }

  public void setNavxAngle(double angle) {
    navx.setNavxAngle(angle);
  } 

  public double getNavxYaw(){
    return navx.currentYaw();
  }

  public double getNavxPitch(){
    return -navx.currentPitch();
  }

  public double getNavxRoll(){
    return navx.currentRoll();
  }

  public double getNavxRate() {
    return navx.getAngleRate();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Roll", getNavxRoll());
    // SmartDashboard.putNumber("Pitch", navx.currentPitch());
    navx.currentPitch();
    // This method will be called once per scheduler run
  }
}
