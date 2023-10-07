// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.MagIntake;

public class AutonomousOuttake extends CommandBase {
  /** Creates a new Outtake. */
  MagIntake mi;
  double start, delay = 0.5, seconds;
  public AutonomousOuttake(MagIntake mi, double seconds) {
    // Seconds = -1 if you don't want it to stop after some amount of time
    this.mi = mi;
    this.seconds = seconds;
    addRequirements(mi);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = Timer.getFPGATimestamp();
    mi.setIntakeDown();
    mi.setFrontMagazine(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp()-start>delay){
      mi.setFrontMagazine(-0.5);
      mi.setIntakePercent(-0.5);
    }
    System.out.println("Outtaking" + (Timer.getFPGATimestamp()-start));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mi.setIntakeUp();
    mi.setFrontMagazine(0);
    mi.setIntakePercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp()-start>seconds){
      return true;
    } else {
      return false;
    }
  }
}
