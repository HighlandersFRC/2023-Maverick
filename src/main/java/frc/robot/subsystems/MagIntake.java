// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.defaults.MagIntakeDefault;
import frc.robot.tools.PneumaticsControl;

public class MagIntake extends SubsystemBase {
  /** Creates a new MagIntake. */

  private final PneumaticsControl pneumatics;
  private double startTime, vibrateTime = 0.5;
  //declare and initialize motors
  private final TalonFX backMagazine = new TalonFX(15, "Canivore");
  private final TalonFX frontMagazine = new TalonFX(14, "Canivore");
  private final TalonFX intakeMotor = new TalonFX(12, "Canivore");
  //declare and initialize configurators
  private TalonFXConfigurator backMagazineConfig = backMagazine.getConfigurator();
  private TalonFXConfigurator frontMagazineConfig = frontMagazine.getConfigurator();
  private TalonFXConfigurator intakeMotorConfig = intakeMotor.getConfigurator();
  //declare and initialize configurations
  private TalonFXConfiguration backMagazineConfigs = new TalonFXConfiguration();
  private MotorOutputConfigs backMagazineMotorOutputConfigs = new MotorOutputConfigs();
  private TalonFXConfiguration frontMagazineConfigs = new TalonFXConfiguration();
  private MotorOutputConfigs frontMagazineMotorOutputConfigs = new MotorOutputConfigs();
  private TalonFXConfiguration intakeMotorConfigs = new TalonFXConfiguration();
  private MotorOutputConfigs intakeMotorMotorOutputConfigs = new MotorOutputConfigs();

  private Logger logger = Logger.getInstance();
  public MagIntake(PneumaticsControl pneumatics) {
    this.pneumatics = pneumatics;
  }
  public void init() {
    //Back Magazine Motor's slot 0 pid
    backMagazineConfigs.Slot0.kP = 0.3;
    backMagazineConfigs.Slot0.kI = 0.0006;
    backMagazineConfigs.Slot0.kD = 0.025;

    //Front Magazine Motor's slot 0 pid
    frontMagazineConfigs.Slot0.kP = 0.2;
    frontMagazineConfigs.Slot0.kI = 0.0;
    frontMagazineConfigs.Slot0.kD = 0.0;

    //Set Brake Mode to Mag Motors
    backMagazineConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    frontMagazineConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //Set Default Command
    setDefaultCommand(new MagIntakeDefault(this));

    // Set update frequencies (Status Frame Period from phoenix 5)
    intakeMotor.getFaultField().setUpdateFrequency(10);
    intakeMotor.getReverseLimit().setUpdateFrequency(10);
    intakeMotor.getForwardLimit().setUpdateFrequency(10);
    intakeMotor.getPosition().setUpdateFrequency(10);
    intakeMotor.getVelocity().setUpdateFrequency(10);
    intakeMotor.getStickyFaultField().setUpdateFrequency(10);
    intakeMotor.getSupplyVoltage().setUpdateFrequency(0.1);
    intakeMotor.getSupplyCurrent().setUpdateFrequency(0.1);
    intakeMotor.getStatorCurrent().setUpdateFrequency(0.1);
    intakeMotor.getMotionMagicIsRunning().setUpdateFrequency(1);
    intakeMotor.getClosedLoopDerivativeOutput().setUpdateFrequency(1);
    intakeMotor.getClosedLoopError().setUpdateFrequency(1);
    intakeMotor.getClosedLoopFeedForward().setUpdateFrequency(1);
    intakeMotor.getClosedLoopSlot().setUpdateFrequency(1);
    intakeMotor.getClosedLoopIntegratedOutput().setUpdateFrequency(1);
    intakeMotor.getClosedLoopOutput().setUpdateFrequency(1);
    intakeMotor.getClosedLoopProportionalOutput().setUpdateFrequency(1);
    intakeMotor.getClosedLoopReference().setUpdateFrequency(1);
    intakeMotor.getClosedLoopReferenceSlope().setUpdateFrequency(1);
    intakeMotor.getRotorPosition().setUpdateFrequency(10);
    intakeMotor.getRotorVelocity().setUpdateFrequency(10);

    frontMagazine.getFaultField().setUpdateFrequency(10);
    frontMagazine.getReverseLimit().setUpdateFrequency(10);
    frontMagazine.getForwardLimit().setUpdateFrequency(10);
    frontMagazine.getPosition().setUpdateFrequency(10);
    frontMagazine.getVelocity().setUpdateFrequency(10);
    frontMagazine.getStickyFaultField().setUpdateFrequency(10);
    frontMagazine.getSupplyVoltage().setUpdateFrequency(0.1);
    frontMagazine.getSupplyCurrent().setUpdateFrequency(0.1);
    frontMagazine.getStatorCurrent().setUpdateFrequency(0.1);
    frontMagazine.getMotionMagicIsRunning().setUpdateFrequency(1);
    frontMagazine.getClosedLoopDerivativeOutput().setUpdateFrequency(1);
    frontMagazine.getClosedLoopError().setUpdateFrequency(1);
    frontMagazine.getClosedLoopFeedForward().setUpdateFrequency(1);
    frontMagazine.getClosedLoopSlot().setUpdateFrequency(1);
    frontMagazine.getClosedLoopIntegratedOutput().setUpdateFrequency(1);
    frontMagazine.getClosedLoopOutput().setUpdateFrequency(1);
    frontMagazine.getClosedLoopProportionalOutput().setUpdateFrequency(1);
    frontMagazine.getClosedLoopReference().setUpdateFrequency(1);
    frontMagazine.getClosedLoopReferenceSlope().setUpdateFrequency(1);
    frontMagazine.getRotorPosition().setUpdateFrequency(10);
    frontMagazine.getRotorVelocity().setUpdateFrequency(10);

    backMagazine.getFaultField().setUpdateFrequency(10);
    backMagazine.getReverseLimit().setUpdateFrequency(10);
    backMagazine.getForwardLimit().setUpdateFrequency(10);
    backMagazine.getPosition().setUpdateFrequency(10);
    backMagazine.getVelocity().setUpdateFrequency(10);
    backMagazine.getStickyFaultField().setUpdateFrequency(10);
    backMagazine.getSupplyVoltage().setUpdateFrequency(0.1);
    backMagazine.getSupplyCurrent().setUpdateFrequency(0.1);
    backMagazine.getStatorCurrent().setUpdateFrequency(0.1);
    backMagazine.getMotionMagicIsRunning().setUpdateFrequency(1);
    backMagazine.getClosedLoopDerivativeOutput().setUpdateFrequency(1);
    backMagazine.getClosedLoopError().setUpdateFrequency(1);
    backMagazine.getClosedLoopFeedForward().setUpdateFrequency(1);
    backMagazine.getClosedLoopSlot().setUpdateFrequency(1);
    backMagazine.getClosedLoopIntegratedOutput().setUpdateFrequency(1);
    backMagazine.getClosedLoopOutput().setUpdateFrequency(1);
    backMagazine.getClosedLoopProportionalOutput().setUpdateFrequency(1);
    backMagazine.getClosedLoopReference().setUpdateFrequency(1);
    backMagazine.getClosedLoopReferenceSlope().setUpdateFrequency(1);
    backMagazine.getRotorPosition().setUpdateFrequency(10);
    backMagazine.getRotorVelocity().setUpdateFrequency(10);

    // apply the configurations
    backMagazineConfig.apply(backMagazineMotorOutputConfigs);
    backMagazineConfig.apply(frontMagazineMotorOutputConfigs);
    backMagazineConfig.apply(intakeMotorMotorOutputConfigs);
    backMagazineConfig.apply(backMagazineConfigs);
    frontMagazineConfig.apply(frontMagazineConfigs);
    intakeMotorConfig.apply(intakeMotorConfigs);
  }

  public double rotateBackMag(double degrees) {
    double revs = (degrees / 360.0) + backMagazine.getPosition().getValue();
    PositionDutyCycle cr = new PositionDutyCycle(revs);
    backMagazine.setControl(cr);
    return revs;
  }

  public double rotateFrontMag(double degrees) {
    double revs = (degrees / 360.0) + backMagazine.getPosition().getValue();
    PositionDutyCycle cr = new PositionDutyCycle(revs);
    frontMagazine.setControl(cr);
    return revs;
  }

  public double getBackMagPosition() {
    return backMagazine.getPosition().getValue();
  }

  public double getFrontMagPosition() {
    return frontMagazine.getPosition().getValue();
  }

  public void setFrontMagRPM(double rpm) {
    VelocityDutyCycle cr = new VelocityDutyCycle(rpm/60);
    frontMagazine.setControl(cr);
  }

  public void setBackMagRPM(double rpm) {
    VelocityDutyCycle cr = new VelocityDutyCycle(rpm/60);
    backMagazine.setControl(cr);
  }

  public double getFrontMagRPM() {
    return frontMagazine.getVelocity().getValue()*60;
  }

  public double getBackMagRPM() {
    return frontMagazine.getVelocity().getValue()*60;
  }

  private final DigitalInput lowerBackBeamBreak = new DigitalInput(1);
  private final DigitalInput upperBeamBreak = new DigitalInput(0);

  public Boolean getLowerBackBeamBreak() {
      return lowerBackBeamBreak.get();
  }

  public Boolean getUpperBeamBreak() {
      return upperBeamBreak.get();
  }

  public void stopMagazine() {
    backMagazine.set(0.0);
    frontMagazine.set(0.0);
  }

  public boolean hasCube(){
    return(frontMagazine.getStatorCurrent().getValue() > 0 && frontMagazine.getStatorCurrent().getValue() < 35);
  }

  public void moveMagazine() {  
      if(!getUpperBeamBreak()){
        stopMagazine();
      } else {
        setFrontMagazine(-0.5);
        // if(!getLowerBackBeamBreak()){
        //     setBackMagazine(0.3);
        //   } else{
        //     setBackMagazine(0.0);
        //   }
      }
  }

  public void setMagazinePercents(double backPercent, double frontPercent) {
    backMagazine.set(backPercent);
    frontMagazine.set(frontPercent);
  }

  public void postGreenWheelVelocity() {
    SmartDashboard.putNumber("GW VELOCITY", getBackMagRPM());
  }

  public void setBackMagazine(double percent) {
    backMagazine.set(percent);
  } 
public void setFrontMagazine(double percent) {
    frontMagazine.set(-percent);
  }

  public void setIntakeUp() {
    pneumatics.setIntakeUp();
  }

  public void setIntakeDown() {
    pneumatics.setIntakeDown();
  }

  public void setIntakePercent(double percent) {
    intakeMotor.set(-percent);
  }

  @Override
  public void periodic() {
    // logging motor current and voltage
    logger.recordOutput("intakeMotorSupply", intakeMotor.getSupplyCurrent().getValue());
    logger.recordOutput("intakeMotorStator", intakeMotor.getSupplyCurrent().getValue());
    logger.recordOutput("intakeMotorVoltage", intakeMotor.getSupplyVoltage().getValue());
    logger.recordOutput("frontMagMotorSupply", frontMagazine.getSupplyCurrent().getValue());
    logger.recordOutput("frontMagMotorStator", frontMagazine.getSupplyCurrent().getValue());
    logger.recordOutput("frontMagMotorVoltage", frontMagazine.getSupplyVoltage().getValue());
    logger.recordOutput("backMagMotorSupply", backMagazine.getSupplyCurrent().getValue());
    logger.recordOutput("backMagMotorStator", backMagazine.getSupplyCurrent().getValue());
    logger.recordOutput("backMagMotorVoltage", backMagazine.getSupplyVoltage().getValue());
    // log beam breaks
    logger.recordOutput("Lower Back Beam Break", getLowerBackBeamBreak());
    logger.recordOutput("Upper Beam Break", getUpperBeamBreak());
    // vibrate controller after intaking a cube
  }

}
