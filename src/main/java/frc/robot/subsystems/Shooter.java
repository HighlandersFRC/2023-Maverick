// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ShooterDefault;

public class Shooter extends SubsystemBase {

 private final TalonFX leftShooter = new TalonFX(10, "Canivore");
 private final TalonFX rightShooter = new TalonFX(9, "Canivore");
 private final TalonFXConfigurator leftShooterConfigurator = leftShooter.getConfigurator();
 private final TalonFXConfigurator rightShooterConfigurator = rightShooter.getConfigurator();
 private final TalonFXConfiguration leftShooterConfiguration = new TalonFXConfiguration();
 private final TalonFXConfiguration rightShooterConfiguration = new TalonFXConfiguration();

 private Peripherals peripherals;

  public Shooter(Peripherals peripherals) {
    this.peripherals = peripherals;
  }

  // method run to set PID values and sets motors to Coast
  public void init() {
    leftShooterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightShooterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightShooterConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightShooterConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 65;
    rightShooterConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -65;
    leftShooterConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 65;
    leftShooterConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -65;

    rightShooterConfiguration.Slot0.kV = 0.06;
    rightShooterConfiguration.Slot0.kP = 0.2;
    rightShooterConfiguration.Slot0.kI = 0;
    rightShooterConfiguration.Slot0.kD = 0;

    Follower follower = new Follower(9, true);
    leftShooter.setControl(follower);
    
    leftShooterConfigurator.apply(leftShooterConfiguration);
    rightShooterConfigurator.apply(rightShooterConfiguration);

    // setDefaultCommand(new ShooterDefault(this, peripherals));
  }

  public double getNumRPMValues(double[] rpmArray) {
    int count = 0;
    for (int i = 0; i < rpmArray.length; i ++) {
      if (!(rpmArray[i] == 0.0)) {
        count ++;
      }
    }
    return count;
  }

  // zeroes shooter encoder
  public void zeroShooterEncoder() {
    leftShooter.setRotorPosition(0);
}

  // sets shooter motor to a specified percent
  public void setShooterPercent(double percent) {
    rightShooter.set(percent);
  }

  // sets shooter to run a velocity PID at given RPM
  public void setShooterRPM(double rpm) {
    VelocityDutyCycle cr = new VelocityDutyCycle(rpm/60);
    rightShooter.setControl(cr);
}

// converts shooter RPM to units/100 ms
public double getShooterRPM(){
  return rightShooter.getVelocity().getValue()*60;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}