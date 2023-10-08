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
import frc.robot.commands.AutonomousFollower;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.PathAuto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveForwardAuto extends PathAuto {
  private File pathingFile;
  private JSONArray pathJSON;
  private JSONObject pathRead;
  /** Creates a new MoveForwardAuto. */
  public MoveForwardAuto(Drive drive, Peripherals peripherals) {
    pathJSON = getPathPoints("/home/lvuser/deploy/IcebabyTestAuto.json");
    addRequirements(drive);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousFollower(drive, pathJSON, true)
    );
  }
  public JSONArray getStartingPath(){
    return pathJSON;
  }
}
