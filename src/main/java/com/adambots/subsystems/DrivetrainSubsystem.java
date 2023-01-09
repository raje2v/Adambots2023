// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.adambots.RobotContainer;
import com.adambots.RobotMap;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.adambots.Constants.DriveConstants;
import com.adambots.Gamepad.Buttons;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DrivetrainSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftEncoderPort,
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftEncoderPort,
      DriveConstants.kRearLeftDriveEncoderReversed,
      DriveConstants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightEncoderPort,
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightEncoderPort,
      DriveConstants.kRearRightDriveEncoderReversed,
      DriveConstants.kRearRightTurningEncoderReversed);

  // The gyro sensor
  private final Gyro m_gyro = RobotMap.GyroSensor;
  private Joystick ex3dPro;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),  
  new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_rearLeft.getPosition(),
    m_rearRight.getPosition()
  });

  public DrivetrainSubsystem(Joystick ex3dPro) {
      this.ex3dPro = ex3dPro;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        // m_frontLeft.getState(),
        // m_frontRight.getState(),
        // m_rearLeft.getState(),  
        // m_rearRight.getState(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

     SmartDashboard.putNumber("getY", ex3dPro.getY());
     SmartDashboard.putNumber("getX", ex3dPro.getX());
     SmartDashboard.putNumber("getZ", ex3dPro.getZ());
     SmartDashboard.putNumber("deadX", RobotContainer.deaden(ex3dPro.getX(), 0.15));
     SmartDashboard.putNumber("deadY", RobotContainer.deaden(ex3dPro.getY(), 0.15));
     SmartDashboard.putNumber("deadZ", RobotContainer.deaden(ex3dPro.getZ(), 0.3));

     SmartDashboard.putNumber("ControllergetY", Buttons.primaryJoystick.getLeftY());
     SmartDashboard.putNumber("ControllergetX", Buttons.primaryJoystick.getLeftX());
     SmartDashboard.putNumber("ControllergetZ", Buttons.primaryJoystick.getRightX());
     SmartDashboard.putNumber("ControllerdeadY", RobotContainer.deaden(Buttons.primaryJoystick.getLeftY(), 0.15));
     SmartDashboard.putNumber("ControllerdeadX", RobotContainer.deaden(Buttons.primaryJoystick.getLeftX(), 0.3));
     SmartDashboard.putNumber("ControllerdeadZ", RobotContainer.deaden(Buttons.primaryJoystick.getRightX(), 0.25));

     SmartDashboard.putNumber("m_frontLeft", m_frontLeft.getState().angle.getDegrees());
     SmartDashboard.putNumber("m_rearLeft", m_rearLeft.getState().angle.getDegrees());
     SmartDashboard.putNumber("m_frontRight", m_frontRight.getState().angle.getDegrees());
     SmartDashboard.putNumber("m_rearRight", m_rearRight.getState().angle.getDegrees());

     SmartDashboard.putNumber("gyro", getHeading());
     
     SmartDashboard.putNumber("m_frontLeft speeed", m_frontLeft.getState().speedMetersPerSecond);
     SmartDashboard.putNumber("m_rearLeft speeed", m_rearLeft.getState().speedMetersPerSecond);
     SmartDashboard.putNumber("m_frontRight speeed", m_frontRight.getState().speedMetersPerSecond);
     SmartDashboard.putNumber("m_rearRight speeed", m_rearRight.getState().speedMetersPerSecond);


     SmartDashboard.putNumber("Odom Rot", m_gyro.getRotation2d().getDegrees());
     SmartDashboard.putNumber("Odom X", m_odometry.getPoseMeters().getX());
     SmartDashboard.putNumber("Odom Y", m_odometry.getPoseMeters().getY());

    //  System.out.println(m_gyro.getRotation2d());
     //  SmartDashboard.putNumber("speedmms",)

     //     System.out.printf("Speed: %f, Turn: %f\n", speedMetersPerSecond, turningRadians);

    //  SmartDashboard.putNumber("getTroddle", ex3dPro.getThrottle());
    //  SmartDashboard.putNumber("getTwist", ex3dPro.getTwist());



  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())

            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    // System.out.printf("XSpeed: %f, YSpeed: %f, Rot: %f\n", xSpeed, ySpeed, rot);
    // System.out.printf("Left Speed: %f, Rot: %f\n", swerveModuleStates[0].speedMetersPerSecond, swerveModuleStates[0].angle.getDegrees());
    /**
     */
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
     m_rearRight.setDesiredState(swerveModuleStates[3]);
    
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setPIDValues(double kP, double kI, double kD) {
    m_frontLeft.setPIDValues(kP, kI, kD);
    m_frontRight.setPIDValues(kP, kI, kD);
    m_rearLeft.setPIDValues(kP, kI, kD);
    m_rearRight.setPIDValues(kP, kI, kD);
  }
}