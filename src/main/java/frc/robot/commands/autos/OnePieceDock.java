// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutonomousFollower;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Peripherals;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceDock extends SequentialCommandGroup {
  private File part1File;
  private JSONArray part1JSON;
  private JSONObject part1Read;
  private File part2File;
  private JSONArray part2JSON;
  private JSONObject part2Read;
  /** Creates a new OnePieceDock. */
  public OnePieceDock(Drive drive, Peripherals peripherals, MagIntake magIntake) {
    try {
      part1File = new File("/home/lvuser/deploy/1PieceDockMaverickPart1.json");
      FileReader scanner = new FileReader(part1File);
      part1Read = new JSONObject(new JSONTokener(scanner));
      part1JSON = (JSONArray) part1Read.get("sampled_points");
      part2File = new File("/home/lvuser/deploy/1PieceDockMaverickPart2.json");
      scanner = new FileReader(part2File);
      part2Read = new JSONObject(new JSONTokener(scanner));
      part2JSON = (JSONArray) part2Read.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }
    addRequirements(drive, magIntake);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousOuttake(magIntake, 1),
      new AutonomousFollower(drive, part1JSON, true),
      new WaitCommand(0.58),
      new AutonomousFollower(drive, part2JSON, true),
      new AutoBalance(drive, peripherals)
    );
  }
}
