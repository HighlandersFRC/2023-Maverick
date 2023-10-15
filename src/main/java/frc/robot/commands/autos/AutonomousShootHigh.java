// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Shooter;

public class AutonomousShootHigh extends CommandBase {
  /** Creates a new Shoot. */
  Shooter shooter;
  MagIntake magIntake;
  double seconds, start;
  public AutonomousShootHigh(Shooter shooter, MagIntake magIntake, double seconds) {
    this.magIntake = magIntake;
    this.shooter = shooter;
    this.seconds = seconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, magIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = Timer.getFPGATimestamp();
    shooter.setShooterPercent(0.18);
    magIntake.setBackMagazine(0.3);
    magIntake.setFrontMagazine(0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterPercent(0);
    magIntake.setBackMagazine(0);
    magIntake.setFrontMagazine(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - start>seconds){
      return true;
    }else {
      return false;
    }
  }
}
