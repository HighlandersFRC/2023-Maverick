// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutonomousFollower;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.PathAuto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceBump extends PathAuto {
  private String part1Path = "/home/lvuser/deploy/3PieceBumpMaverickPart1.json";
  private JSONArray part1JSON;
  private JSONObject part1Read;

  private String part2Path = "/home/lvuser/deploy/3PieceBumpMaverickPart2.json";
  private JSONArray part2JSON;
  private JSONObject part2Read;

  /** Creates a new ThreePieceBlueBump. */
  public ThreePieceBump(Drive drive, Peripherals peripherals, MagIntake magIntake) {
    part1JSON = getPathPoints(part1Path);
    part2JSON = getPathPoints(part2Path);

    addRequirements(drive, magIntake);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousOuttake(magIntake, 1),
      new ParallelCommandGroup(
        new AutonomousFollower(drive, part1JSON, true),
        new SequentialCommandGroup(
          new WaitCommand(1.75),
          new AutonomousIntake(magIntake, 1.75)
        )
      ),
      new AutonomousOuttake(magIntake, 1)
    );
  }

  @Override
  public JSONArray getStartingPath() {
    return part1JSON;
  }
}
