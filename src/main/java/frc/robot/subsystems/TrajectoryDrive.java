// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class TrajectoryDrive extends SubsystemBase{
    private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(new _righttDriveTalon(DriveConstants.RightDriveTalonPort),(new leftDriveTalon(DriveConstants.LeftDriveTalonPort));
    private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(new _righttDriveTalon(DriveConstants.RightDriveTalonPort),(new leftDriveTalon(DriveConstants.LeftDriveTalonPort));

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    private final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    private final Encoder m_rightEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);

    private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);
    private final DifferentialDriveOdometry m_odometry;

    public TrajectoryDrive(){
        m_leftEncoder.setDistancePerPulse(distancePerPulse);
        m_rightMotors.setDistancePerPulse(distancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }

    private void resetEncoders() {
    }



}
