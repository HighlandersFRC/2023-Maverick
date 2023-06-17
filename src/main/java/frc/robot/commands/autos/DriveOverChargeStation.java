package frc.robot.commands.autos;

import frc.robot.tools.math.Vector;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;

public class DriveOverChargeStation extends CommandBase {
  private Peripherals peripherals;
  private Drive drive;
  private boolean pastStation = false;
  private boolean checkpoint = false;
  private boolean exitingStation = false;
  private double timePastStation = 0;

  Vector driveVector = new Vector(2, 0);
  Vector stopVector = new Vector(0.0, 0.0);

  public DriveOverChargeStation(Drive drive, Peripherals peripherals) {
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
    SmartDashboard.putBoolean("checkpoint", checkpoint);
    SmartDashboard.putBoolean("pastStation", pastStation);
    SmartDashboard.putNumber("timePastStation", timePastStation);
    SmartDashboard.putBoolean("peripherals null", this.peripherals == null);
    if(this.peripherals != null) {
      if(peripherals.getNavxRollOffset() > 10) {
        checkpoint = true;
      }

      if(peripherals.getNavxRollOffset() < -5 && checkpoint && !exitingStation) {
        exitingStation = true;
      }

      if(peripherals.getNavxRollOffset() > -5 && exitingStation && !pastStation) {
        timePastStation = Timer.getFPGATimestamp();
        pastStation = true;
      }

    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.autoRobotCentricDrive(stopVector, 0);
  }

  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - timePastStation > 0.3 && pastStation) {
      return true;
    } else {
      return false;
    }
  }
}
