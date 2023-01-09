
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots;

import com.adambots.Gamepad.Buttons;
import com.adambots.Gamepad.GamepadConstants;
import com.adambots.commands.*;
import com.adambots.commands.autonCommands.*;
import com.adambots.subsystems.*;
import com.adambots.utils.Log;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  // private final TankDriveTrainSubsystem driveTrainSubsystem = new TankDriveTrainSubsystem(RobotMap.GyroSensor, 
  //                                                                                 RobotMap.GearShifter, 
  //                                                                                 RobotMap.FrontRightMotor, 
  //                                                                                 RobotMap.FrontLeftMotor, 
  //                                                                                 RobotMap.BackLeftMotor, 
  //                                                                                 RobotMap.BackRightMotor);
  
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
    dash();
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

  private void dash(){
    // autoChooser.setDefaultOption("None", null);
    // autoChooser.addOption("Auton1Ball", new Auton1Ball(catapultSubsystem, driveTrainSubsystem));
    // autoChooser.addOption("Auton2Ball", new Auton2Ball(driveTrainSubsystem, intakeSubsystem, catapultSubsystem));
    // autoChooser.addOption("Position1Auton3Ball", new Position1Auton3Ball(driveTrainSubsystem, intakeSubsystem, catapultSubsystem));
    // autoChooser.addOption("Position1Auton5Ball", new Position1Auton5Ball(driveTrainSubsystem, intakeSubsystem, catapultSubsystem));
    // autoChooser.addOption("Position2Auton4Ball", new Position2Auton4Ball(driveTrainSubsystem, intakeSubsystem, catapultSubsystem));

    SmartDashboard.putData("Auton Mode", autoChooser);
  }

  private void setupDefaultCommands(){
    // driveTrainSubsystem.setDefaultCommand(
    //    new DriveCommand(driveTrainSubsystem, 
    //    () -> deaden(-Buttons.primaryJoystick.getLeftY()),
    //     () -> Buttons.primaryJoystick.getRightX())
    //     );  
  }

  // deadzoning
  public double deaden(double rawInput) {
    return Math.abs(rawInput) < GamepadConstants.DEADZONE ? 0 : rawInput;
  }

  public static double deaden(double input, double sadDeadenVariable){
    if(Math.abs(input) < sadDeadenVariable){
        return 0;
    }else{
        return input;
    }
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
      
    // return new Auton2Ball(driveTrainSubsystem, intakeSubsystem, catapultSubsystem);
    // System.out.println(autoChooser.getSelected().toString());
    return autoChooser.getSelected();

    // return new LowerIntakeArmCommand(intakeSubsystem)
    // .andThen(new WaitCommand(4))
    // .andThen(new TurnToAngleFromCameraCommand(driveTrainSubsystem))
    // .andThen(new DriveToBallCommand(driveTrainSubsystem, intakeSubsystem, conveyorSubsystem, RobotMap.IntakePhotoEye));
    // .andThen(new StartIntakeCommand(intakeSubsystem, () -> -1.0))
    // .andThen(new DriveForwardDistanceCommand(driveTrainSubsystem, 20000, -Constants.AUTON_DRIVE_FORWARD_SPEED))
  }
}
