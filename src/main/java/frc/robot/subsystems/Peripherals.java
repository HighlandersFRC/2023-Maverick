// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
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

  private Pigeon2 pigeon = new Pigeon2(0, "Canivore");

  private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();

  public void init(){
    // values from pigeon calibration in phoenix tuner
    pigeonConfig.MountPose.MountPosePitch = 85.20455932617188;
    pigeonConfig.MountPose.MountPoseRoll = 131.78761291503906;
    pigeonConfig.MountPose.MountPoseYaw = 5.973172187805176;
    pigeon.getConfigurator().apply(pigeonConfig);
    // zeroNavx();
    zeroPigeon();  
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

  //pigeon
  double pitchOffset = getRawPitch();
  double rollOffset = getRawRoll();

  public void zeroPigeon(){
    setYaw(0.0);
  }

  public void setYaw(double degrees){
    pigeon.setYaw(degrees);
  }

  public double getYaw(){
    return pigeon.getYaw().getValue();
  }

  public void setPitch(double degrees){
    pitchOffset = degrees-getRawPitch();
  }

  public double getRawPitch(){
    return pigeon.getPitch().getValue();
  }

  public double getPitch(){
    return pigeon.getPitch().getValue()+pitchOffset;
  }

  public void setRoll(double degrees){
    pitchOffset = degrees-getRawRoll();
  }

  public double getRawRoll(){
    return pigeon.getRoll().getValue();
  }

  public double getRoll(){
    return pigeon.getRoll().getValue()+rollOffset;
  }

  public double getZAcceleration(){
    return pigeon.getAccelerationZ().getValue();
  }

  public double getXAcceleration(){
    return pigeon.getAccelerationX().getValue();
  }

  public double getYAcceleration(){
    return pigeon.getAccelerationY().getValue();
  }

  public double getZAccumulation(){
    return pigeon.getAccumGyroZ().getValue();
  }

  public double getXAccumulation(){
    return pigeon.getAccumGyroX().getValue();
  }

  public double getYAccumulation(){
    return pigeon.getAccumGyroY().getValue();
  }

  public double getYawVelocity(){
    return pigeon.getAngularVelocityZ().getValue();
  }

  public double getPitchVelocity(){
    return pigeon.getAngularVelocityY().getValue();
  }

  public double getRollVelocity(){
    return pigeon.getAngularVelocityX().getValue();
  }

  public double getGravityVectorX(){
    return pigeon.getGravityVectorX().getValue();
  }

  public double getGravityVectorY(){
    return pigeon.getGravityVectorY().getValue();
  }

  public double getGravityVectorZ(){
    return pigeon.getGravityVectorZ().getValue();
  }

  public double getMagneticFieldX(){
    return pigeon.getMagneticFieldX().getValue();
  }

  public double getMagneticFieldY(){
    return pigeon.getMagneticFieldY().getValue();
  }

  public double getMagneticFieldZ(){
    return pigeon.getMagneticFieldZ().getValue();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Roll", getNavxRoll());
    SmartDashboard.putNumber("Pigeon Yaw", getYaw());
    SmartDashboard.putNumber("Pigeon Pitch", getPitch());
    SmartDashboard.putNumber("Pigeon Roll", getRoll());
    // SmartDashboard.putNumber("Pitch", navx.currentPitch());
    navx.currentPitch();
    // This method will be called once per scheduler run
  }
}
