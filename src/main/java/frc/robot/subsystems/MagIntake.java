// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.hardware.DeviceIdentifier;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.MagIntakeDefault;
import frc.robot.tools.PneumaticsControl;

public class MagIntake extends SubsystemBase {
  /** Creates a new MagIntake. */

  private final PneumaticsControl pneumatics;

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

    // frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
    // frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);
    // frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000);
    // frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10000);
    // frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000);
    // frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 1000);
    // frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000);
    // frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000);
    // frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000);
  
    // backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
    // backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);
    // backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000);
    // backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10000);
    // backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000);
    // backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 1000);
    // backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000);
    // backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000);
    // backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000);

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

  public void moveMagazine() {  
      if(!getUpperBeamBreak()){
        stopMagazine();
      } else {
        setFrontMagazine(-0.5);
        if(!getLowerBackBeamBreak()){
            setBackMagazine(0.3);
          } else{
            setBackMagazine(0.0);
          }
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
    // This method will be called once per scheduler run
  }

}
