// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Lights.LEDMode;

public class Intake extends CommandBase {
  /** Creates a new Outtake. */
  MagIntake mi;
  double start, delay = 0.5;
  double startTime, vibrateTime = 2;
  Lights lights;
  Logger logger = Logger.getInstance();
  public Intake(MagIntake mi, Lights lights) {
    // Seconds = -1 if you don't want it to stop after some amount of time
    this.mi = mi;
    this.lights = lights;
    addRequirements(mi, lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = Timer.getFPGATimestamp();
    mi.setIntakeDown();
    mi.setFrontMagazine(0.1);
    logger.recordOutput("Intaking?", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((Timer.getFPGATimestamp()-start)>delay){
      mi.setFrontMagazine(0.5);
      mi.setIntakePercent(0.5);
    }
    System.out.println("Intaking" + (Timer.getFPGATimestamp()-start));
    if (mi.hasCube()){
      if (startTime == 0){
        startTime = Timer.getFPGATimestamp();
      }
      if (Timer.getFPGATimestamp()-startTime>vibrateTime){
        OI.driverController.setRumble(RumbleType.kBothRumble, 0.5);
        OI.operatorController.setRumble(RumbleType.kBothRumble, 0.5);
        lights.setMode(LEDMode.COLOR1STROBE);
      } else {
        OI.driverController.setRumble(RumbleType.kBothRumble, 0);
        OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
        lights.setMode(lights.getDefaultMode());
      }
    } else {
      startTime = 0;
      OI.driverController.setRumble(RumbleType.kBothRumble, 0);
      OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
      lights.setMode(lights.getDefaultMode());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mi.setIntakeUp();
    mi.setFrontMagazine(0);
    mi.setIntakePercent(0);
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
    logger.recordOutput("Intaking?", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !OI.driverRT.getAsBoolean();
  }
}
