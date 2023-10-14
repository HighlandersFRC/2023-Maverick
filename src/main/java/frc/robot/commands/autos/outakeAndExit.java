// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.ZeroNavx;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.MagIntake;
import frc.robot.tools.Vector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class outakeAndExit extends SequentialCommandGroup {
  /** Creates a new outakeAndExit. */
  MagIntake magIntake;
  Drive drive;
  Vector vector = new Vector(1, 0);
  public outakeAndExit(MagIntake magIntake, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.magIntake = magIntake;
    this.drive = drive;
    addRequirements(magIntake, drive);
    addCommands(
      new ZeroNavx(drive),
      new AutonomousOuttake(magIntake, 2),
      new AutonomousDrive(drive, vector, 3.5)
    );
  }
}
