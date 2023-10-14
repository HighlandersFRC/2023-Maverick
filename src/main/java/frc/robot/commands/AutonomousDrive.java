// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.tools.Vector;

public class AutonomousDrive extends CommandBase {
  /** Creates a new AutonomousDrive. */
  Drive drive;
  double seconds, start;
  Vector vector;
  public AutonomousDrive(Drive drive, Vector vector, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.seconds = seconds;
    this.vector = vector;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setWheelsStraight();
    start = Timer.getFPGATimestamp();
    drive.autoDrive(vector, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp()-start > seconds){
      return true;
    } else {
      return false;
    }
  }
}
