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
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Peripherals;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceBump extends SequentialCommandGroup {
  private File part1File;
  private JSONArray part1JSON;
  private JSONObject part1Read;

  private File part2File;
  private JSONArray part2JSON;
  private JSONObject part2Read;

  private File part3File;
  private JSONArray part3JSON;
  private JSONObject part3Read;

  private File part4File;
  private JSONArray part4JSON;
  private JSONObject part4Read;

  /** Creates a new ThreePieceBlueBump. */
  public ThreePieceBump(Drive drive, Peripherals peripherals, MagIntake magIntake) {
    try {
      part1File = new File("/home/lvuser/deploy/1PieceDockMaverickPart1.json");
      FileReader scanner = new FileReader(part1File);
      part1Read = new JSONObject(new JSONTokener(scanner));
      part1JSON = (JSONArray) part1Read.get("sampled_points");

      part2File = new File("/home/lvuser/deploy/1PieceDockMaverickPart2.json");
      scanner = new FileReader(part2File);
      part2Read = new JSONObject(new JSONTokener(scanner));
      part2JSON = (JSONArray) part2Read.get("sampled_points");

      part3File = new File("/home/lvuser/deploy/1PieceDockMaverickPart3.json");
      scanner = new FileReader(part3File);
      part3Read = new JSONObject(new JSONTokener(scanner));
      part3JSON = (JSONArray) part3Read.get("sampled_points");

      part4File = new File("/home/lvuser/deploy/1PieceDockMaverickPart4.json");
      scanner = new FileReader(part4File);
      part4Read = new JSONObject(new JSONTokener(scanner));
      part4JSON = (JSONArray) part4Read.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    addRequirements(drive, magIntake);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    );
  }
}
