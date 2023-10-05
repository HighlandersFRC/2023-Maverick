// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.MoveWheelToAngle;
import frc.robot.commands.Outtake;
import frc.robot.commands.ZeroNavx;
import frc.robot.commands.autos.MoveForwardAuto;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.PneumaticsControl;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Peripherals peripherals = new Peripherals();
  public Drive drive = new Drive(peripherals);
  private Logger logger = Logger.getInstance();
  private PneumaticsControl pneumatics = new PneumaticsControl();
  private MagIntake magIntake = new MagIntake(pneumatics);
  private Lights lights = new Lights();

  File pathingFile;
  String pathString;

  JSONObject pathRead;
  JSONArray pathJSON;

  // String fieldSide;

  SequentialCommandGroup auto;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    drive.init();
    peripherals.init();
    logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs/mostRecent.wpilog"));
    logger.addDataReceiver(new NT4Publisher());
    logger.start();

    try {
      pathingFile = new File("/home/lvuser/deploy/IcebabyTestAuto.json");
      FileReader scanner = new FileReader(pathingFile);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
    } catch (Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }
    this.auto = new MoveForwardAuto(drive, peripherals);
    auto.schedule();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    peripherals.getNavxAngle();
    logger.recordOutput("Swerve Module States", drive.getModuleStates());
    logger.recordOutput("Swerve Module Setpoints", drive.getModuleSetpoints());
    logger.recordOutput("Angle Motor Velocity", drive.getAngleMotorVelocity());
    logger.recordOutput("Navx", Math.toRadians(peripherals.getNavxAngle()));
    logger.recordOutput("Odometry", drive.getOdometry());
    // logger.recordOutput("Y Value", drive.getFusedOdometryY());
    // logger.recordOutput("Theta Value", drive.getFusedOdometryTheta());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    try {
      this.auto.schedule();
    } catch (Exception e){
      System.out.println("No auto is selected");
    } 
    drive.autoInit(this.pathJSON);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    OI.buttonA.whileTrue(new MoveWheelToAngle(drive, 0.5));
    OI.buttonB.whileTrue(new MoveWheelToAngle(drive, -0.5));
    OI.rt.whileTrue(new IntakeBalls(magIntake, lights));
    OI.lt.whileTrue(new Outtake(magIntake));
    OI.viewButton.whileTrue(new ZeroNavx(drive));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
