// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class PIDDriveForward extends CommandBase {
  /** Creates a new PIDDriveForward. */
  private final DriveTrain driveTrain;
  private double distance;
  private double speed;
  private double error;

  public PIDDriveForward(DriveTrain dt, double d, double s) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    distance = d;
    speed = s;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = distance - driveTrain.getDistance();
    error = (error / distance);
    speed = error * 0.7;

    if (speed > 0.7){
      speed = 0.7;
    }
    if (speed < 0.1){
      speed = 0.1;
    }
    driveTrain.tankDrive(0, 0);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   System.out.println("Current Position: " + driveTrain.getDistance());
   return (driveTrain.getDistance() >= distance);
  }
}
