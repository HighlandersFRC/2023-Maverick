// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.node.BooleanNode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ClimberDefault;
import frc.robot.tools.PneumaticsControl;

public class Climber extends SubsystemBase {
  private final CANSparkMax rotatingMotor = new CANSparkMax(22, MotorType.kBrushless);
  private double maxPos = Constants.getClimberFalconTics(30);
  private PneumaticsControl pneumatics;
  private Logger log = Logger.getInstance();

  public Climber(PneumaticsControl pneumaticsControl) {
    this.pneumatics = pneumaticsControl;
  }

  public void init() {
    setDefaultCommand(new ClimberDefault(this));
    rotatingMotor.restoreFactoryDefaults();
    rotatingMotor.getEncoder().setPosition(0);
    rotatingMotor.setSmartCurrentLimit(13, 15);

    rotatingMotor.getPIDController().setP(0.05, 0);
    rotatingMotor.getPIDController().setI(0, 0);
    rotatingMotor.getPIDController().setD(0.1, 0);

    rotatingMotor.getPIDController().setP(0.0001, 1);
    rotatingMotor.getPIDController().setI(0,1 );
    rotatingMotor.getPIDController().setD(0.1, 1);
    rotatingMotor.getPIDController().setFF(0, 1);
    rotatingMotor.getPIDController().setSmartMotionMaxAccel(1500, 1);
    rotatingMotor.getPIDController().setSmartMotionMaxVelocity(3000, 1);

    // rotatingMotor.getPIDController().setSmartMotionMinOutputVelocity(100, 1);

    // rotatingMotor.getEncoder().setPosition(0);

    // climberFalcon1.s
  }

  public double getRotatingMotorPosition() {
    return Constants.getRotatingClimberAngle(rotatingMotor.getEncoder().getPosition());
  }

  public double getRotatingClimberCurrent() {
    return rotatingMotor.getOutputCurrent();
  }

  public void zeroRotatingMotor() {
    rotatingMotor.getEncoder().setPosition(0);
  }

  public void setRotatingMotorPosition(double degrees) {
    double tics = Constants.getNeoTics(degrees);
    rotatingMotor.getPIDController().setReference(tics, ControlType.kPosition);
  }

    public void setRotatingMotorMagic(double degrees) {
    double ticks = Constants.getNeoTics(degrees);
    rotatingMotor.getPIDController().setReference(ticks, ControlType.kSmartMotion, 1);
  }

  public void setRotatingMotorPercent(double percent) {
    rotatingMotor.set(percent);
  }

  public void postRotatingClimberEncoder() {
    SmartDashboard.putNumber("ROTATING", getRotatingMotorPosition());
    SmartDashboard.putNumber("RTICS", rotatingMotor.getEncoder().getPosition());
  }

  public double getRotatingMotorTemperature() {
    return rotatingMotor.getMotorTemperature();
  }

  public Boolean getLimitSwitch() {
    return rotatingMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
  }

  // public Boolean getClimberLimitSwitch() {
  //   return rotatingMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
  // }

  @Override
  public void periodic() {}
}
