// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class FireOneBall extends SequentialCommandGroup {
  /** Creates a new FireOneBall. */
  private ShotAdjuster adjuster;
  public FireOneBall(Drive drive, MagIntake magIntake, Shooter shooter, Hood hood, Peripherals peripherals, Lights lights, double hoodPosition, double shooterRPM, double firstBallTimeout, double secondBallTimeout, ShotAdjuster shotAdjuster, double offset, Boolean useList) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.adjuster = shotAdjuster;
    addRequirements(drive, magIntake, shooter, hood);
    // System.out.println("")
    // shooterRPM = shooterRPM + adjuster.getRPMAdjustment();
    // hoodPosition = hoodPosition + adjuster.getHoodAdjustment();
    addCommands(
      new ParallelCommandGroup(
          new SpinShooter(shooter, peripherals, shooterRPM, adjuster, useList),
          new FaceTarget(drive, peripherals, lights, offset),
          // new VisionAlignment(drive, peripherals),
          new SetHoodPosition(hood, peripherals, hoodPosition, adjuster, useList)
      ),
      // new FaceTarget(drive, peripherals, offset),
      new TurnBackMag(magIntake, 720)
    );
  }
}
