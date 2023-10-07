package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.tools.Vector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;


public class AutonomousBalance extends CommandBase {

  private Peripherals peripherals;
  private Drive drive;
  private boolean end = false;
  Vector closerBalanceVector = new Vector(0, -0.25);
  Vector fartherBalanceVector = new Vector(0, 0.25);
  Vector stopVector = new Vector(0.0, 0.0);

  Vector balanceVector;
  public AutonomousBalance(Drive drive, Peripherals peripherals) {
    this.drive = drive;
    this.peripherals = peripherals;
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {    
    if (peripherals.getNavxRollOffset()<-10){
      drive.autoRobotCentricDrive(closerBalanceVector, 0);
    } else if (peripherals.getNavxRollOffset()>10){
      drive.autoRobotCentricDrive(fartherBalanceVector, 0);
    } else {
      drive.autoRobotCentricDrive(stopVector, 0);
      end = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.autoRobotCentricDrive(stopVector, 0);
  }

  @Override
  public boolean isFinished() {
    return end;
  }
}