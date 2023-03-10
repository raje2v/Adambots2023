/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.subsystems;

import com.adambots.Constants.DriveConstants;
import com.adambots.sensors.Gyro;
import com.adambots.utils.Log;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveTrainSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveTrainNew.
   */
  private Solenoid gearShifter;

  private WPI_TalonFX frontRightMotor;
  private WPI_TalonFX frontLeftMotor;
  private WPI_TalonFX backLeftMotor;
  private WPI_TalonFX backRightMotor;

  DifferentialDrive drive;

  private double speedModifier;

  private Gyro gyro;

  private boolean hasGyroBeenReset = false;

  public TankDriveTrainSubsystem(Gyro gyro, Solenoid gearShifter, WPI_TalonFX frontRightMotor, WPI_TalonFX frontLeftMotor, WPI_TalonFX backLeftMotor, WPI_TalonFX backRightMotor) {
    super();

    this.gyro = gyro;

    speedModifier = DriveConstants.kNormalSpeedModifier;

    this.gearShifter = gearShifter;

    this.frontRightMotor = frontRightMotor; 
    this.frontLeftMotor = frontLeftMotor; 

    this.backLeftMotor = backLeftMotor; 
    this.backRightMotor = backRightMotor; 

    this.backLeftMotor.follow(frontLeftMotor);
    this.backRightMotor.follow(frontRightMotor);

    this.backLeftMotor.setInverted(false);
    this.frontLeftMotor.setInverted(false);

    this.backRightMotor.setInverted(true); // Right motors are reversed in our Robot
    this.frontRightMotor.setInverted(true);

    frontLeftMotor.configOpenloopRamp(DriveConstants.kNeutralToFull);
    frontRightMotor.configOpenloopRamp(DriveConstants.kNeutralToFull);

    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);

    drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

    Log.info("Initializing Drive Subsystem");
  }

  public void resetEncoders() {

    Log.info("Resetting encoders");

    frontLeftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, DriveConstants.kDrivePIDSlot, 0);
    frontLeftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    frontRightMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    // BackLeftMotor.getSensorCollection().setQuadraturePosition(0, 0);
    // BackRightMotor.getSensorCollection().setQuadraturePosition(0, 0);
  }

  public double getAverageDriveEncoderValue() {
    double averageEncoderPos = (Math
        .abs(frontLeftMotor.getSelectedSensorPosition()) + Math.abs(frontRightMotor.getSelectedSensorPosition())) / 2;
    return averageEncoderPos;
  }

  public double getLeftDriveEncoderVelocity() {
    return frontLeftMotor.getSelectedSensorVelocity();
  }

  public double getRightDriveEncoderVelocity() {
    return frontRightMotor.getSelectedSensorVelocity();
  }

  public double getLeftDistanceInch(){
    return frontLeftMotor.getSelectedSensorPosition() * DriveConstants.kEncoderTickerPerInch;
  }

  public double getRightDistanceInch(){
    return frontRightMotor.getSelectedSensorPosition() * DriveConstants.kEncoderTickerPerInch;
  }

  public void setLowSpeed() {
    speedModifier = DriveConstants.kLowSpeedModifier;
  }

  public void setNormalSpeed() {
    speedModifier = DriveConstants.kNormalSpeedModifier;
  }

  public void arcadeDrive(double speed, double turnSpeed) {
    int frontRobotDirection = -1;
    double straightSpeed = frontRobotDirection * speed * speedModifier;
    SmartDashboard.putNumber("Yaw", gyro.getYaw());

    // Log.infoF("Arcade Drive - Straight Speed = %f - Turn Speed = %f - Gyro Angle = %f", straightSpeed, turnSpeed * speedModifier, gyro.getAngle());
    drive.arcadeDrive(straightSpeed, turnSpeed * speedModifier);
  }

  public void shiftHighGear() {

    Log.info("Shifting to high gear");
    gearShifter.set(true);
  }

  public void shiftLowGear() {

    Log.info("Shifting to low gear");
    gearShifter.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Get yaw - -180 to 180 range
   * 
   * @return
   */
  public double getYaw() {
    return gyro.getYaw();
  }

  /**
   * Get cummulative yaw - can be greater than 360
   * 
   * @return
   */
  public double getAngle() {
    return gyro.getAngle();
  }

  public void resetGyro(boolean force) {
    if (!hasGyroBeenReset || force) {
      gyro.reset();
      Log.info("Gyro has been reset");
      System.out.println("Gyro has been reset");
      hasGyroBeenReset = true;
    }
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading(){
    double heading = gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    // double heading = Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.GYRO_REVERSED ? -1.0 : 1.0);
  
    // System.out.println("Heading: " + heading);
    return heading;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    resetGyro(false);
  }

  public void resetGyro(){
    Log.info("Gyro has been reset");

    resetGyro(false);
  }
}
