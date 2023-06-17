package frc.robot.commands.autos;

import frc.robot.tools.math.Vector;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;

public class DriveBackOnChargeStation extends CommandBase {
  private Peripherals peripherals;
  private Drive drive;
  private boolean checkpoint = false;
  private double startTimeOnStation = 0;
  private boolean balanced = false;

  Vector driveVector = new Vector(-1.75, 0);
  Vector balanceVector = new Vector(-0.4, 0.0);
  Vector stopVector = new Vector(0.0, 0.0);

  public DriveBackOnChargeStation(Drive drive, Peripherals peripherals) {
    this.drive = drive;
    this.peripherals = peripherals;
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {
    drive.autoRobotCentricDrive(driveVector, 0);
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("checkpoint 2", checkpoint);
    if(this.peripherals != null) {
      if(peripherals.getNavxRollOffset() < -10 && !checkpoint) {
        checkpoint = true;
        startTimeOnStation = Timer.getFPGATimestamp();
      }

      if(checkpoint && Timer.getFPGATimestamp() - startTimeOnStation > 0.8) {
        drive.autoRobotCentricDrive(balanceVector, 0);
      }

      if(checkpoint && peripherals.getNavxRollOffset() > -10 && !balanced) {
        balanced = true;
      }

    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.autoRobotCentricDrive(stopVector, 0);
  }

  @Override
  public boolean isFinished() {
    return balanced;
  }
}
