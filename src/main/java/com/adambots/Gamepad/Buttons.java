/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.Gamepad;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * All Game Controller Button Mappings
 */
public class Buttons {

    // initialize controllers
    public static final CommandXboxController primaryJoystick = new CommandXboxController(GamepadConstants.PRIMARY_DRIVER);
    public static final CommandXboxController secondaryJoystick = new CommandXboxController(GamepadConstants.SECONDARY_DRIVER);

    //primary buttons
    public static final Trigger primaryBackButton = primaryJoystick.back();
    public static final Trigger primaryStartButton = primaryJoystick.start();
    public static final Trigger primaryXButton = primaryJoystick.x();
    public static final Trigger primaryYButton = primaryJoystick.y();
    public static final Trigger primaryBButton = primaryJoystick.b();
    public static final Trigger primaryAButton = primaryJoystick.a();
    public static final Trigger primaryLB = primaryJoystick.leftBumper();
    public static final Trigger primaryRB = primaryJoystick.rightBumper();
    public static final Trigger primaryLeftStickButton = primaryJoystick.leftStick();
    public static final Trigger primaryRightStickButton = primaryJoystick.rightStick();
    
    //primary DPad
    public static final Trigger primaryDPadN = primaryJoystick.pov(GamepadConstants.DPAD_N_ANGLE);
    public static final Trigger primaryDPadNW = primaryJoystick.pov(GamepadConstants.DPAD_NW_ANGLE);
    public static final Trigger primaryDPadW = primaryJoystick.pov(GamepadConstants.DPAD_W_ANGLE);
    public static final Trigger primaryDPadSW = primaryJoystick.pov(GamepadConstants.DPAD_SW_ANGLE);
    public static final Trigger primaryDPadS = primaryJoystick.pov(GamepadConstants.DPAD_S_ANGLE);
    public static final Trigger primaryDPadSE = primaryJoystick.pov(GamepadConstants.DPAD_SE_ANGLE);
    public static final Trigger primaryDPadE = primaryJoystick.pov(GamepadConstants.DPAD_E_ANGLE);
    public static final Trigger primaryDPadNE = primaryJoystick.pov(GamepadConstants.DPAD_NE_ANGLE);
    
    //primary axes
    //RIGHT TRIGGER       primaryJoystick.getRightTriggerAxis()
    //LEFT TRIGGER        primaryJoystick.getLeftTriggerAxis()
    //LEFT STICK X AXIS   primaryJoystick.getLeftX()
    //LEFT STICK Y AXIS   primaryJoystick.getLeftY()
    //RIGHT STICK X AXIS  primaryJoystick.getRightX()
    //RIGHT STICK Y AXIS  primaryJoystick.getRightY()
    
    //secondary buttons
    public static final Trigger secondaryBackButton = secondaryJoystick.back();
    public static final Trigger secondaryStartButton = secondaryJoystick.start();
    public static final Trigger secondaryXButton = secondaryJoystick.x();
    public static final Trigger secondaryYButton = secondaryJoystick.y();
    public static final Trigger secondaryBButton = secondaryJoystick.b();
    public static final Trigger secondaryAButton = secondaryJoystick.a();
    public static final Trigger secondaryLB = secondaryJoystick.leftBumper();
    public static final Trigger secondaryRB = secondaryJoystick.rightBumper();
    public static final Trigger secondaryLeftStickButton = secondaryJoystick.leftStick();
    public static final Trigger secondaryRightStickButton = secondaryJoystick.rightStick();
    
    //secondary DPad
    public static final Trigger secondaryDPadN = secondaryJoystick.pov(GamepadConstants.DPAD_N_ANGLE);
    public static final Trigger secondaryDPadNW = secondaryJoystick.pov(GamepadConstants.DPAD_NW_ANGLE);
    public static final Trigger secondaryDPadW = secondaryJoystick.pov(GamepadConstants.DPAD_W_ANGLE);
    public static final Trigger secondaryDPadSW = secondaryJoystick.pov(GamepadConstants.DPAD_SW_ANGLE);
    public static final Trigger secondaryDPadS = secondaryJoystick.pov(GamepadConstants.DPAD_S_ANGLE);
    public static final Trigger secondaryDPadSE = secondaryJoystick.pov(GamepadConstants.DPAD_SE_ANGLE);
    public static final Trigger secondaryDPadE = secondaryJoystick.pov(GamepadConstants.DPAD_E_ANGLE);
    public static final Trigger secondaryDPadNE = secondaryJoystick.pov(GamepadConstants.DPAD_NE_ANGLE);
    
    //secondary axes    
    //RIGHT TRIGGER       secondaryJoystick.getRightTriggerAxis()
    //LEFT TRIGGER        secondaryJoystick.getLeftTriggerAxis()
    //LEFT STICK X AXIS   secondaryJoystick.getLeftX()
    //LEFT STICK Y AXIS   secondaryJoystick.getLeftY()
    //RIGHT STICK X AXIS  secondaryJoystick.getRightX()
    //RIGHT STICK Y AXIS  secondaryJoystick.getRightY()
}
