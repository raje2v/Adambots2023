
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import java.util.List;

import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Gamepad.Buttons;
import com.adambots.commands.*;
import com.adambots.commands.autonCommands.*;
import com.adambots.subsystems.*;
import com.adambots.utils.Log;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // subsystems
  // private final TankDriveTrainSubsystem driveTrainSubsystem = new
  // TankDriveTrainSubsystem(RobotMap.GyroSensor,
  // RobotMap.GearShifter,
  // RobotMap.FrontRightMotor,
  // RobotMap.FrontLeftMotor,
  // RobotMap.BackLeftMotor,
  // RobotMap.BackRightMotor);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(RobotMap.frontLeftSwerveModule,
      RobotMap.rearLeftSwerveModule, RobotMap.frontRightSwerveModule, RobotMap.rearRightSwerveModule);
  // commands
  // private SequentialCommandGroup autonDriveForwardGyroDistanceCommand;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    setupDefaultCommands();

    // Don't enable this for the competition
    // Log.saveToFile("/home/lvuser/robot.txt");

    // Configure the button bindings
    configureButtonBindings();

    // configure the dashboard
    setupDashboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Buttons.primaryAButton.onTrue(command);
    // Buttons.secondaryDPadE.onTrue(command);

    // Buttons.primaryBackButton.whileTrue(command);
  }

  private void setupDashboard() {
    // autoChooser.setDefaultOption("None", null);
    // autoChooser.addOption("Auton1Ball", new Auton1Ball(catapultSubsystem,
    // driveTrainSubsystem));
    // autoChooser.addOption("Auton2Ball", new Auton2Ball(driveTrainSubsystem,
    // intakeSubsystem, catapultSubsystem));
    // autoChooser.addOption("Position1Auton3Ball", new
    // Position1Auton3Ball(driveTrainSubsystem, intakeSubsystem,
    // catapultSubsystem));
    // autoChooser.addOption("Position1Auton5Ball", new
    // Position1Auton5Ball(driveTrainSubsystem, intakeSubsystem,
    // catapultSubsystem));
    // autoChooser.addOption("Position2Auton4Ball", new
    // Position2Auton4Ball(driveTrainSubsystem, intakeSubsystem,
    // catapultSubsystem));

    SmartDashboard.putData("Auton Mode", autoChooser);
  }

  /**
   * This method will be called periodically to update the dashboard values
   * Call from Robot.java robotPeriodic
   */
  public void updateDashboard() {
    SmartDashboard.putNumber("getY", Buttons.forward);
    SmartDashboard.putNumber("getX", Buttons.sideways);
    SmartDashboard.putNumber("getZ", Buttons.rotate);
  }

  private void setupDefaultCommands() {

    drivetrainSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drivetrainSubsystem.drive(
                Buttons.forward,
                Buttons.sideways,
                Buttons.rotate,
                false),
            drivetrainSubsystem));

    // Use the Lambda version with Supplier prefix when passed to a custom command

    // driveTrainSubsystem.setDefaultCommand(
    // // The left stick controls translation of the robot.
    // // Turning is controlled by the X axis of the right stick.
    // new DriveCommand(

    // drivetrainSubsystem.drive(
    // Buttons.forwardSupplier,
    // Buttons.sidewaysSupplier,
    // Buttons.rotateSupplier,
    // false),
    // DrivetrainSubsystem
    // ));

  }

  /**
   * Use this to pass the autonomo
   * us command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // if (autoChooser.getSelected() != null)
    // Log.info("Chosen Auton Command: ", autoChooser.getSelected().toString());
    // else
    // Log.info("Chosen Auton Command: None");

    // return new Auton2Ball(driveTrainSubsystem, intakeSubsystem,
    // catapultSubsystem);
    // System.out.println(autoChooser.getSelected().toString());
    // return autoChooser.getSelected();

    // return new LowerIntakeArmCommand(intakeSubsystem)
    // .andThen(new WaitCommand(4))
    // .andThen(new TurnToAngleFromCameraCommand(driveTrainSubsystem))
    // .andThen(new DriveToBallCommand(driveTrainSubsystem, intakeSubsystem,
    // conveyorSubsystem, RobotMap.IntakePhotoEye));
    // .andThen(new StartIntakeCommand(intakeSubsystem, () -> -1.0))
    // .andThen(new DriveForwardDistanceCommand(driveTrainSubsystem, 20000,
    // -Constants.AUTON_DRIVE_FORWARD_SPEED))

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(/* new Translation2d(1, 1), new Translation2d(2, -1) */),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1, 0, new Rotation2d(3)),
        config);
    // System.out.println("Total time: " + exampleTrajectory.getTotalTimeSeconds());
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    CustomSwerveControllerCommand swerveControllerCommand = new CustomSwerveControllerCommand(
        exampleTrajectory,
        new Pose2d(1, 0, new Rotation2d(3)),
        drivetrainSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drivetrainSubsystem::setModuleStates,
        drivetrainSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivetrainSubsystem.drive(0, 0, 0, false));
  }
}
