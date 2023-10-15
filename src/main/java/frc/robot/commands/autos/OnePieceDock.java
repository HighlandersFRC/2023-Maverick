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
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.AutonomousFollower;
import frc.robot.commands.SetNavxValue;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.PathAuto;
import frc.robot.tools.Vector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceDock extends PathAuto {
  private JSONArray part1JSON;
  private JSONObject part1Read;
  private String part1Path = "/home/lvuser/deploy/1PieceDockMaverickPart1.json";
  private JSONArray part2JSON;
  private JSONObject part2Read;
  private String part2Path = "/home/lvuser/deploy/1PieceDockMaverickPart2.json";
  /** Creates a new OnePieceDock. */
  public OnePieceDock(Drive drive, Shooter shooter, Peripherals peripherals, MagIntake magIntake) {
    part1Read = getPathJSONObject(part1Path);
    part1JSON = getPathPoints(part1Path);
    part2Read = getPathJSONObject(part2Path);
    part2JSON = getPathPoints(part2Path);
    Vector forwardVector = new Vector(2, 0);
    Vector backwardVector = new Vector(-2, 0);
    addRequirements(drive, magIntake);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetNavxValue(peripherals, 180),
      new AutonomousShootHigh(shooter, magIntake, 2),
      // new AutonomousRotate(drive, -35),
      new AutonomousDrive(drive, forwardVector, 4),
      // new AutonomousRotate(drive, 35),
      new AutonomousDrive(drive, backwardVector, 2),
      new AutonomousBalance(drive, peripherals)
    );
  }
  @Override
  public JSONArray getStartingPath(){
    return part1JSON;
  }
  @Override
  public String getName(){
    return "One Piece Dock";
  }
}
