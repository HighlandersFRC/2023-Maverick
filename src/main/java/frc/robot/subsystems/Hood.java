// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.HoodDefaultCommand;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */

  private TalonFX hoodMotor = new TalonFX(13);
  private TalonFXConfigurator hoodConfigurator = hoodMotor.getConfigurator();
  private TalonFXConfiguration hoodConfiguration = new TalonFXConfiguration();

  private Peripherals peripherals;

  public Hood(Peripherals peripherals) {
    this.peripherals = peripherals;
  }

  // method run to set PID values and configure motor settings
  public void init() {
    // hoodMotor.config_kP(0, 0.2);
    // hoodMotor.config_kI(0, 0.00003);
    // hoodMotor.config_kD(0, 1);
    // hoodMotor.config_IntegralZone(0, 0.01);
    hoodConfiguration.Slot0.kP = 0.19;
    hoodConfiguration.Slot0.kI = 0.00002;
    hoodConfiguration.Slot0.kD = 1;
    hoodConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 65;
    hoodConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -65;
    
    hoodConfiguration.HardwareLimitSwitch.ReverseLimitEnable = true;
    hoodConfiguration.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    hoodConfiguration.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;

    setDefaultCommand(new HoodDefaultCommand(this, peripherals));

  }

  public void setHoodPercent(double percent) {
    // System.out.println("PERCENT: " + percent);
    hoodMotor.set(percent);
  }

  public boolean getLowerLimitSwitch() {
    if(hoodMotor.getReverseLimit().getValue().value == 1) {
      return false;
    }
    return true;
  }

  public double getSensorPosition() {
    return hoodMotor.getRotorPosition().getValue();
  }

  public double getHoodPosition() {
    return getSensorPosition();
  }

  public void zeroHood() {
    hoodMotor.setRotorPosition(0);
  }

  public void setHoodPosition(double position) {
    PositionDutyCycle cr = new PositionDutyCycle(position);
    hoodMotor.setControl(cr);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
