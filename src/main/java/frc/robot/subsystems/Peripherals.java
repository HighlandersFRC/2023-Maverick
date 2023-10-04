// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Tools.Navx;

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

  public void zeroNavx(){
    navx.softResetYaw();
    navx.softResetAngle();
  }

  public void setNavxAngle(double angle) {
    navx.setNavxAngle(angle);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
