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

import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.MoveWheelToAngle;
import frc.robot.commands.Outtake;
import frc.robot.commands.Intake;
import frc.robot.commands.ZeroNavx;
import frc.robot.commands.autos.AutonomousBalance;
import frc.robot.commands.autos.AutonomousIntake;
import frc.robot.commands.autos.AutonomousOuttake;
import frc.robot.commands.autos.AutonomousRotate;
import frc.robot.commands.autos.MoveForwardAuto;
import frc.robot.commands.autos.OnePieceDock;
import frc.robot.commands.autos.ThreePieceBump;
import frc.robot.commands.autos.ThreePieceFeeder;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.PathAuto;
import frc.robot.tools.PneumaticsControl;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private PathAuto m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private SendableChooser<PathAuto> autoChooser = new SendableChooser<>();
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

  String fieldSide;
  SendableChooser<String> sideChooser = new SendableChooser<>();

  PathAuto auto;
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

    autoChooser.setDefaultOption("One Piece Dock", new OnePieceDock(drive, peripherals, magIntake));
    autoChooser.addOption("3 Piece Feeder", new ThreePieceFeeder(drive, magIntake, peripherals));
    autoChooser.addOption("3 Piece Bump", new ThreePieceBump(drive, peripherals, magIntake));
    autoChooser.addOption("Test Auto", new MoveForwardAuto(drive, peripherals));
    SmartDashboard.putData(autoChooser);

    sideChooser.setDefaultOption("Blue", "blue");
    sideChooser.addOption("Red", "red");
    SmartDashboard.putData(sideChooser);
    // if (autoChooser.getSelected().equals("One Piece Dock")){
      
    //}
    // HI
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
    //Logging Swerve Module States
    logger.recordOutput("Swerve Module States", drive.getModuleStates());
    logger.recordOutput("Swerve Module Setpoints", drive.getModuleSetpoints());
    // Logging NavX
    logger.recordOutput("Navx", Math.toRadians(peripherals.getNavxAngle()));
    // Logging Odometry
    logger.recordOutput("Odometry", drive.getOdometry());
    // Logging chosen auto
    logger.recordOutput("Chosen Auto", autoChooser.getSelected().getName());
    // Logging fieldside
    logger.recordOutput("fieldSide", fieldSide);
    // Logging Compressor Current
    logger.recordOutput("Compressor Current", pneumatics.getCompressorCurrent());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // if (autoChooser.getSelected().getName() == "One Piece Dock"){
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/IcebabyTestAuto.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // } else if (autoChooser.getSelected().getName() == "3 Piece Red Feeder"){
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/3PieceFeederMaverickPart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // }else if (autoChooser.getSelected().getName() == "3 Piece Red Bump"){
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/3PieceBumpMaverickPart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // }else if (autoChooser.getSelected().getName() == "3 Piece Blue Bump"){
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/3PieceBumpMaverickPart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // }else if (autoChooser.getSelected().getName() == "3 Piece Blue Feeder"){
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/3PieceFeederMaverickPart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // }

    // m_autonomousCommand = autoChooser.getSelected();

    // schedule the autonomous command (example)
    this.auto = autoChooser.getSelected();
    auto.schedule();
    this.pathJSON = auto.getStartingPath();
    this.fieldSide = sideChooser.getSelected();
    drive.setFieldSide(fieldSide);
    drive.autoInit(this.pathJSON, this.fieldSide);
    logger.recordOutput("pathJSON", pathJSON.toString());
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

    OI.driverA.whileTrue(new MoveWheelToAngle(drive, 0.5));
    OI.driverB.whileTrue(new MoveWheelToAngle(drive, -0.5));
    OI.driverY.whileTrue(new AutonomousRotate(drive, 30));
    OI.driverRT.whileTrue(new Intake(magIntake));
    OI.driverLT.whileTrue(new Outtake(magIntake));
    // OI.operatorRT.whileTrue(new Intake(magIntake));
    // OI.operatorLT.whileTrue(new Outtake(magIntake));
    OI.driverView.whileTrue(new ZeroNavx(drive));
    OI.driverX.onTrue(new AutonomousBalance(drive, peripherals));
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
