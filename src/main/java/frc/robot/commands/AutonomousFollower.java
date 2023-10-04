// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import org.json.JSONArray;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Tools.Vector;
import frc.robot.subsystems.Drive;

public class AutonomousFollower extends CommandBase {
  private Drive drive;

  private JSONArray path;
  private JSONArray commands;

  private double initTime;
  private double currentTime;
  private double previousTime;

  private double odometryFusedX = 0;
  private double odometryFusedY = 0;
  private double odometryFusedTheta = 0;

  private double[] desiredVelocityArray = new double[3];
  private double desiredThetaChange = 0;

  private boolean record;
  private String fieldSide;
  private int rowOffset = 0;

  private ArrayList<double[]> recordedOdometry = new ArrayList<double[]>();
  private boolean generatePath = false;
  /** Creates a new AutonomousFollower. */
  public AutonomousFollower(Drive drive, JSONArray pathPoints, boolean record) {
    this.drive = drive;
    this.path = pathPoints;
    this.record = record;
    this.generatePath = false;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    odometryFusedX = drive.getFusedOdometryX();
    odometryFusedY = drive.getFusedOdometryY();
    odometryFusedTheta = drive.getFusedOdometryTheta();

    // System.out.println("X: " + odometryFusedX + " Y: " + odometryFusedY + " Theta: " + odometryFusedTheta);

    currentTime = Timer.getFPGATimestamp() - initTime;
    
    // call PIDController function
    desiredVelocityArray = drive.pidController(odometryFusedX, odometryFusedY, odometryFusedTheta, currentTime, path);
    
    // create velocity vector and set desired theta change
    Vector velocityVector = new Vector();
    velocityVector.i = desiredVelocityArray[0];
    velocityVector.j = desiredVelocityArray[1];
    desiredThetaChange = desiredVelocityArray[2];

    drive.autoDrive(velocityVector, desiredThetaChange);

    previousTime = currentTime;

    if (this.record){
      recordedOdometry.add(new double[] {currentTime, odometryFusedX, odometryFusedY, odometryFusedTheta});
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Vector velocityVector = new Vector();
    velocityVector.i = 0.0;
    velocityVector.j = 0.0;
    double desiredThetaChange = 0.0;
    drive.autoDrive(velocityVector, desiredThetaChange);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(currentTime > path.getJSONArray(path.length() - 1).getDouble(0)) {
      return true;
    }
    else {
      return false;
    }
  }
}

