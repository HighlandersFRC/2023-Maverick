// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.FireBallsNoVision;
import frc.robot.commands.IntakeBalls;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.ShotAdjuster;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallNew extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuto. */
  private ShotAdjuster shotAdjuster = new ShotAdjuster();
  private File pathingFile;
  private JSONArray pathJSON;
  private JSONObject pathRead;

  private File pathingFile2;
  private JSONArray pathJSON2;
  private JSONObject pathRead2;

  public TwoBallNew(Drive drive, Peripherals peripherals, Lights lights, MagIntake magIntake, Shooter shooter, Hood hood) {

    try {
      pathingFile = new File("/home/lvuser/deploy/2BallPart1.json");
      FileReader scanner = new FileReader(pathingFile);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile2 = new File("/home/lvuser/deploy/2BallPart2.json");
      FileReader scanner2 = new FileReader(pathingFile2);
      pathRead2 = new JSONObject(new JSONTokener(scanner2));
      pathJSON2 = (JSONArray) pathRead2.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    addRequirements(drive);
    addCommands(
      new ParallelDeadlineGroup(new AutonomousFollower(drive, pathJSON, false), new IntakeBalls(magIntake, lights)),
      new WaitCommand(0.1),
      new ParallelDeadlineGroup(new WaitCommand(1), new FireBallsNoVision(drive, magIntake, shooter, hood, peripherals, lights, 25, 1500, 0.5, 0, shotAdjuster)),
      new WaitCommand(0.1),
      new ParallelDeadlineGroup(new AutonomousFollower(drive, pathJSON2, false), new IntakeBalls(magIntake, lights)),
      new WaitCommand(0.1),
      new ParallelDeadlineGroup(new WaitCommand(1), new FireBallsNoVision(drive, magIntake, shooter, hood, peripherals, lights, 25, 1500, 0.5, 0, shotAdjuster))
    );
  }
}
