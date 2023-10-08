// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import org.json.JSONArray;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousFollower;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.PathAuto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceFeeder extends PathAuto {
  /** Creates a new ThreePieceRedFeeder. */
  String part1Path = "/home/lvuser/deploy/3PieceFeederMaverickPart1.json", part2Path = "/home/lvuser/deploy/3PieceFeederMaverickPart2.json";
  JSONArray part1Array, part2Array;
  public ThreePieceFeeder(Drive drive, MagIntake magIntake, Peripherals peripherals) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    part1Array = getPathPoints(part1Path);
    part2Array = getPathPoints(part2Path);
    addCommands(
      new AutonomousOuttake(magIntake, 1),
      new ParallelCommandGroup(
        new AutonomousIntake(magIntake, 5), 
        new AutonomousFollower(drive, part1Array, true)
      ),
      new AutonomousOuttake(magIntake, 1)
      // new ParallelCommandGroup(
      //   new AutonomousIntake(magIntake, 7), 
      //   new AutonomousFollower(drive, part2Array, true)
      // ),
      // new AutonomousOuttake(magIntake, 1)
    );
  }
  public JSONArray getStartingPath(){
    return part1Array;
  }
}
