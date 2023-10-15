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
public class OnePieceDock extends SequentialCommandGroup {
  private JSONArray part1JSON;
  /** Creates a new OnePieceDock. */
  public OnePieceDock(Drive drive, Shooter shooter, Peripherals peripherals, MagIntake magIntake) {
    Vector forwardVector = new Vector(2, 0);
    Vector backwardVector = new Vector(-1, 0);
    addRequirements(drive, magIntake);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetNavxValue(peripherals, -90),
      // new AutonomousRotate(drive, 35),
      new AutonomousDrive(drive, backwardVector, 0.5),
      new AutonomousDrive(drive, forwardVector, 2),
      new AutonomousBalance(drive, peripherals)
    );
  }
}
