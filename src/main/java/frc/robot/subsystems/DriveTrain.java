// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private final WPI_TalonSRX _leftDriveTalon;
  private final WPI_TalonSRX _rightDriveTalon;

  private double circumference = 3.81; //converted 1.5 inches to centimeters

  private DifferentialDrive _diffDrive;

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
  _leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
  _rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);

  _leftDriveTalon.setInverted(false);
  _rightDriveTalon.setInverted(false);
  
  _diffDrive = new DifferentialDrive(_leftDriveTalon, _rightDriveTalon);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    _diffDrive.tankDrive(leftSpeed, rightSpeed);
  }


public double getDistance() {
	return ((_leftDriveTalon.getSelectedSensorPosition(0) + _rightDriveTalon.getSelectedSensorPosition(0) / 2) * ((circumference/4096.0)));
}
public void resetEncoders(){
  _leftDriveTalon.setSelectedSensorPosition(0, 0, 10);
  _rightDriveTalon.setSelectedSensorPosition(0, 0, 10);
}
public void gyroReset(){
  navx.reset();
}
public double getAngle(){
  return getAngle();
}

    
}
