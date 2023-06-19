package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ClimbSequence;
import frc.robot.commands.ClimbSequenceP2;
import frc.robot.commands.ContinuousAccelerationInterpolation;
import frc.robot.commands.FaceTarget;
import frc.robot.commands.FireBalls;
import frc.robot.commands.FireBallsNoVision;
import frc.robot.commands.FireOneBall;
import frc.robot.commands.HoldRotatingArmOnBar;
import frc.robot.commands.HubCentricAutoRanging;
import frc.robot.commands.HubCentricDrive;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.LoadedRobotClimb;
import frc.robot.commands.LockDriveWheels;
import frc.robot.commands.Outtake;
import frc.robot.commands.PositionRotatingClimber;
import frc.robot.commands.PositionVerticalClimber;
import frc.robot.commands.ResetAutoOdometry;
import frc.robot.commands.ResetClimber;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.RobotCentricDrive;
import frc.robot.commands.RunRotatingClimber;
import frc.robot.commands.RunVerticalClimber;
import frc.robot.commands.TurnDriveTrain;
import frc.robot.commands.ZeroNavxMidMatch;
import frc.robot.commands.autos.AutoBalance;
import frc.robot.commands.autos.Chezy2BallSteal;
import frc.robot.commands.autos.Chezy5BallAuto;
import frc.robot.commands.autos.FiveBallAuto;
import frc.robot.commands.autos.OneBallAuto;
import frc.robot.commands.autos.OnePieceDock;
import frc.robot.commands.autos.OutAndTurnAuto;
import frc.robot.commands.autos.ShortLineAuto;
import frc.robot.commands.autos.SquareDemo;
import frc.robot.commands.autos.TestAuto;
import frc.robot.commands.autos.ThreeBallAuto;
import frc.robot.commands.autos.ThreePieceMav;
import frc.robot.commands.autos.TwoBallAuto;
import frc.robot.commands.autos.TwoBallNew;
import frc.robot.commands.autos.TwoBallSteal;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MagIntake;
// import frc.robot.subsystems.MqttPublish;
// import frc.robot.subsystems.MqttSubscribe;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Lights.LEDMode;
import frc.robot.tools.PneumaticsControl;
import frc.robot.tools.ShotAdjuster;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.System.Logger.Level;

import javax.swing.text.Position;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  File f;
  BufferedWriter bw;
  FileWriter fw;
  double startTime;

  File pathingFile;
  String pathString;
  JSONArray pathJSON;

  private File pathingFile2;
  private JSONArray pathJSON2;
  private JSONObject pathRead2;

  private File pathingFile3;
  private JSONArray pathJSON3;
  private JSONObject pathRead3;

  private File pathingFile4;
  private JSONArray pathJSON4;
  private JSONObject pathRead4;

  ContinuousAccelerationInterpolation testPath;

  private final PneumaticsControl pneumatics = new PneumaticsControl();
 
  private final String subCameraTopic = "/sensors/camera";

  // private MqttPublish publish = new MqttPublish();
  // private MqttSubscribe subscribe = new MqttSubscribe();

  private final Climber climber = new Climber(pneumatics);
  private final Carriage carriage = new Carriage(pneumatics);

  private Lights lights = new Lights();

  private final Peripherals peripherals = new Peripherals(/*subscribe,*/ lights);
  private final Drive drive = new Drive(peripherals/*, publish*/);
  private final Shooter shooter = new Shooter(peripherals);
  private final Hood hood  = new Hood(peripherals);

  private MagIntake magIntake = new MagIntake(pneumatics);

  private OnePieceDock onePieceDock = new OnePieceDock(drive, peripherals);

  private ThreePieceMav threeBallMav = new ThreePieceMav(drive, peripherals, lights, magIntake, shooter, hood);

  private TwoBallNew twoBallNew = new TwoBallNew(drive, peripherals, lights, magIntake, shooter, hood);

  private OutAndTurnAuto outAndTurnAuto = new OutAndTurnAuto(drive, peripherals, magIntake, lights);

  private ShortLineAuto shortLineAuto = new ShortLineAuto(drive, peripherals);

  private ThreeBallAuto threeBallAuto = new ThreeBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private OneBallAuto oneBallAuto = new OneBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private TwoBallAuto twoBallAuto = new TwoBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  private FiveBallAuto fiveBallAuto = new FiveBallAuto(drive, magIntake, shooter, hood, peripherals, lights);

  // private TwoBallSteal twoBallSteal1 = new TwoBallSteal(drive, magIntake, shooter, hood, peripherals, lights);

  private TwoBallSteal twoBallSteal1 = new TwoBallSteal(drive, magIntake, shooter, hood, peripherals, lights);

  private SquareDemo squareDemo = new SquareDemo(drive, magIntake, shooter, hood, peripherals, lights);

  private UsbCamera cameraBack;
  private UsbCamera cameraFront;
  private VideoSink server;
  private JSONObject pathRead;

  private ShotAdjuster shotAdjuster = new ShotAdjuster();
  private Button whenPressed;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    if (OI.isRedSide()) {
      System.out.println("ON RED SIDE");
      drive.setFieldSide("red");
    } else if (OI.isBlueSide()) {
      System.out.println("ON BLUE SIDE");
      drive.setFieldSide("blue");
    }
    cameraBack = CameraServer.startAutomaticCapture("Back Cam", "/dev/video0");
    cameraBack.setResolution(320, 240);
    cameraBack.setFPS(15);
    SmartDashboard.putNumber("Lights", 0);

    cameraFront = CameraServer.startAutomaticCapture("Front Cam", "/dev/video1");
    cameraFront.setResolution(320, 240);
    cameraFront.setFPS(15);

    server = CameraServer.addSwitchedCamera("driverVisionCameras");  
    server.setSource(cameraFront);
    // System.out.println("###########");
    peripherals.init();
    magIntake.init();
    shooter.init();
    hood.init();
    lights.init();
    climber.init();

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);

    // publish.publish(pubCameraTopic);
    // subscribe.subscribe(subCameraTopic);
    // m_robotContainer = new RobotContainer();


  //   else if(OI.is2BallAuto()) {
  //     try {
  //       pathingFile = new File("/home/lvuser/deploy/2BallAuto.json");
  //       FileReader scanner = new FileReader(pathingFile);
  //       pathJSON = new JSONArray(new JSONTokener(scanner));
  //       // System.out.println(pathJSON);
  //     }
  //     catch(Exception e) {
  //       // System.out.println("ERROR WITH PATH FILE " + e);
  //     }
  //   }
  //   else if(OI.is3BallAuto()) {
  //     try {
  //       pathingFile = new File("/home/lvuser/deploy/5BallPart1.json");
  //       FileReader scanner = new FileReader(pathingFile);
  //       pathJSON = new JSONArray(new JSONTokener(scanner));
  //       // System.out.println(pathJSON);
  //     }
  //     catch(Exception e) {
  //       // System.out.println("ERROR WITH PATH FILE " + e);
  //     }
  //   }
  //   else if(OI.is5BallAuto()) {
  //     try {
  //       pathingFile = new File("/home/lvuser/deploy/5BallPart1.json");
  //       FileReader scanner = new FileReader(pathingFile);
  //       pathJSON = new JSONArray(new JSONTokener(scanner));
  //       // System.out.println(pathJSON);
  //     }
  //     catch(Exception e) {
  //       // System.out.println("ERROR WITH PATH FILE " + e);
  //     }
  //   }
  //   else if(OI.is2BallSteal1()) {
  //     try {
  //       pathingFile = new File("/home/lvuser/deploy/2BallSteal1.json");
  //       FileReader scanner = new FileReader(pathingFile);
  //       pathJSON = new JSONArray(new JSONTokener(scanner));
  //       // System.out.println(pathJSON);
  //     }
  //     catch(Exception e) {
  //       // System.out.println("ERROR WITH PATH FILE " + e);
  //     }
  //   }
  //   else {
  //     try {
  //       pathingFile = new File("/home/lvuser/deploy/5BallTogether.json");
  //       FileReader scanner = new FileReader(pathingFile);
  //       pathJSON = new JSONArray(new JSONTokener(scanner));
  //       // System.out.println(pathJSON);
  //     }
  //     catch(Exception e) {
  //       // System.out.println("ERROR WITH PATH FILE " + e);
  //     }
  //   }

  //   // System.out.println(pathJSON);
    drive.init();
    carriage.init();
  }

  @Override
  public void robotPeriodic() {
    lights.periodic();
    // SmartDashboard.putNumber("Lime Light X", peripherals.getLimeLightX());
    // SmartDashboard.putNumber("Lime Light Y", peripherals.getLimeLightY());
    SmartDashboard.putNumber("Navx angle", peripherals.getNavxAngle());
    SmartDashboard.putNumber("Navx Roll Offset", peripherals.getNavxRollOffset());
    SmartDashboard.putNumber("Navx Roll", peripherals.getNavxRoll());
    CommandScheduler.getInstance().run();

    // SmartDashboard.putBoolean("LIMIT SWITCH", hood.getLowerLimitSwitch());
    // SmartDashboard.putNumber("RPM", shooter.getShooterRPM());
    // SmartDashboard.putNumber("HOOD", hood.getHoodPosition());
    // SmartDashboard.putNumber("RPM ADJUSTMENT", shotAdjuster.getRPMAdjustment());
    // SmartDashboard.putNumber("HOOD ADJUSTMENT", shotAdjuster.getHoodAdjustment());

    // SmartDashboard.putBoolean("BOTTOM BEAM BREAK", magIntake.getLowerBackBeamBreak());
    // SmartDashboard.putBoolean("UPPER BEAM BREAK", magIntake.getUpperBeamBreak());

    // SmartDashboard.putNumber("DISTANCE", peripherals.getLimeLightDistanceToTarget());

    // // climber.postRotatingClimberEncoder();

    // SmartDashboard.putNumber("VClimber", carriage.getclimberFalcon1Position());
    // SmartDashboard.putNumber("RCLIMBER", climber.getRotatingMotorPosition());

    // SmartDashboard.putBoolean("CLIMB LIMIT SWITCH", climber.getLimitSwitch());
    // SmartDashboard.putNumber("ROLL", peripherals.getNavxPitch());

    // SmartDashboard.putNumber("ROTATING TEMPERATURE", climber.getRotatingMotorTemperature());

    magIntake.postGreenWheelVelocity();
  }
  

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    try {
      if(bw != null){ 
        bw.close();
      }
      if(fw != null){
        fw.close();
      }
     
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    // peripherals.turnLightRingOff();
    
  }

  @Override
  public void disabledPeriodic() {
    // peripherals.turnLightRingOff();
  }

  @Override
  public void autonomousInit() {
    if(OI.isShortLineAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/shortlinetest.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
        System.out.println(pathJSON);
        // System.out.println("it worked\n it json path is " + pathJSON + "\n.");
      }
      catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
        System.out.println("task was failed\n json path failed with exception: " + e + "\nthats too bad");
      }
    } else if (OI.isOutAndTurnAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/OutAndTurn.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
        System.out.println(pathJSON);
        // System.out.println("it worked\n it json path is " + pathJSON + "\n.");
      }
      catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
        System.out.println("task was failed\n json path failed with exception: " + e + "\nthats too bad");
      }
    } else if(OI.isTwoBallNew()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/2BallPart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
        System.out.println(pathJSON);
        // System.out.println("it worked\n it json path is " + pathJSON + "\n.");
      }
      catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
        System.out.println("task was failed\n json path failed with exception: " + e + "\nthats too bad");
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
    } else if (OI.is3BallMav()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/3PieceMav1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      }
      catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
  
      try {
        pathingFile2 = new File("/home/lvuser/deploy/3PieceMav2.json");
        FileReader scanner2 = new FileReader(pathingFile2);
        pathRead2 = new JSONObject(new JSONTokener(scanner2));
        pathJSON2 = (JSONArray) pathRead2.get("sampled_points");
      }
      catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
  
      try {
        pathingFile3 = new File("/home/lvuser/deploy/3PieceMav3.json");
        FileReader scanner3 = new FileReader(pathingFile3);
        pathRead3 = new JSONObject(new JSONTokener(scanner3));
        pathJSON3 = (JSONArray) pathRead3.get("sampled_points");
      }
      catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
      
      try {
        pathingFile4 = new File("/home/lvuser/deploy/3PieceMav4.json");
        FileReader scanner4 = new FileReader(pathingFile4);
        pathRead4 = new JSONObject(new JSONTokener(scanner4));
        pathJSON4 = (JSONArray) pathRead4.get("sampled_points");
      }
      catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    }

    // System.out.println(pathJSON);
    if(!OI.is1PieceDock()) {
    drive.autoInit(pathJSON);
    }
    // peripherals.init();
    // testPath = new ContinuousAccelerationInterpolation(drive, pathJSON);
    // testPath.schedule();
    if(OI.isShortLineAuto()) {
      shortLineAuto.schedule();
    } else if(OI.isOutAndTurnAuto()) {
      outAndTurnAuto.schedule();
    } else if (OI.isTwoBallNew()) {
      twoBallNew.schedule();
    } else if (OI.is3BallMav()) {
      threeBallMav.schedule();
    } else if (OI.is1PieceDock()) {
      onePieceDock.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    startTime = Timer.getFPGATimestamp();
    // try{
    //   f = new File("/home/lvuser/Navx/NavxValues" +  (int) startTime + ".csv");
    //   if(!f.exists()){
    //     f.createNewFile();  
    //   }
    //   fw = new FileWriter(f);
    //   bw = new BufferedWriter(fw);
    //   bw.write("started collecting \n" );
      
      
    //   }
    //   catch (Exception e){
    //     e.printStackTrace();
    //   }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    OI.driverRT.whileTrue(new IntakeBalls(magIntake, lights));
    OI.driverLT.whileTrue(new Outtake(magIntake));

    OI.driverA.whenHeld(new FireBallsNoVision(drive, magIntake, shooter, hood, peripherals, lights, 0.0, 1400, 0.75, 0.75, shotAdjuster));
    OI.driverB.whenHeld(new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 10, 1400, 0.75, 0.75, shotAdjuster, 0, true));
    OI.driverY.whenHeld(new FireBallsNoVision(drive, magIntake, shooter, hood, peripherals, lights, 9.25, 1400, 0.5, 0.5, shotAdjuster));
    OI.driverX.whenHeld(new FireBallsNoVision(drive, magIntake, shooter, hood, peripherals, lights, 30, 1650, 0.75, 0.75, shotAdjuster));

    OI.driverRB.whenHeld(new HubCentricAutoRanging(drive, hood, shooter, shotAdjuster, peripherals));
    OI.driverLB.whileTrue(new AutoBalance(drive, peripherals));
    OI.driverViewButton.whileHeld(new ZeroNavxMidMatch(drive));

    OI.operatorA.whenPressed(new LoadedRobotClimb(carriage, 0));
    OI.operatorY.whenPressed(new PositionVerticalClimber(carriage, climber, 21));
    OI.operatorX.whenPressed(new ClimbSequence(climber, carriage));
    OI.operatorB.whenPressed(new HoldRotatingArmOnBar(climber, 30, peripherals));

    OI.operatorViewButton.whenPressed(new ResetClimber(climber, carriage));

    // OI.operatorRT.whileHeld(new RunVerticalClimber(carriage, 0.15));
    // OI.operatorLT.whileHeld(new RunVerticalClimber(carriage, -0.15));
    OI.operatorRB.whileHeld(new RunRotatingClimber(climber, 0.1));
    OI.operatorLB.whileHeld(new RunRotatingClimber(climber, -0.35));

    OI.operatorMenuButton.whileHeld(new RobotCentricDrive(drive));

    OI.driverMenuButton.whileHeld(new ResetOdometry(drive, 7.5, 1.67));

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double[] location = peripherals.calculateRobotPosFromCamera();
    SmartDashboard.putNumber("Controller Y", OI.getDriverLeftY());
    SmartDashboard.putNumber("Controller X", OI.getDriverLeftX());
    SmartDashboard.putNumber("RPM", shooter.getShooterRPM());

    SmartDashboard.putNumber("RobotPosCamX", location[0]);
    SmartDashboard.putNumber("RobotPosCamY", location[1]);
    SmartDashboard.putNumber("MAG RPM", magIntake.getBackMagRPM());


    // if (OI.getDriverA()) {
    //   System.out.println("+");
    // }

    if(OI.getPOV() == 0) {
      shotAdjuster.increaseRPM();
    }
    else if(OI.getPOV() == 180) {
      shotAdjuster.decreaseRPM();
    }
    else {

    }

    if(OI.getPOV() == 270) {
      shotAdjuster.increaseHood();
    }
    else if(OI.getPOV() == 90) {
      shotAdjuster.decreaseHood();
    }
    else {

    }

    //System.out.println("RIGHT --> " + climber.getRightClimberPosition());
    //System.out.println("LEFT --> " + climber.getLeftClimberPosition());
  }

  @Override
  public void testInit() {
    OI.driverRT.whileTrue(new IntakeBalls(magIntake, lights));
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
