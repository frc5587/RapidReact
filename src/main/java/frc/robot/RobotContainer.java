// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.frc5587.lib.control.DeadbandXboxController;

import frc.robot.subsystems.ClimberArm;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  private final DeadbandXboxController xboxController = new DeadbandXboxController(1);
  // Subsystems
  private final ClimberArm climberArm = new ClimberArm();  
  // Commands
  // Others

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Instantiate Y-button on XboxController
    JoystickButton yButton = new JoystickButton(xboxController, XboxController.Button.kY.value);
    // Instantiate left trigger on XboxController
    Trigger leftTrigger = new Trigger(() -> xboxController.getLeftTriggerAxis() > 0);

    /*
     * Climber Arm
     */

     /**
      * When the Y-button and left trigger are active, run the motor.
      * When they are not active, stop the motor.
      */
     yButton.and(leftTrigger.negate()).whenActive(climberArm::on, climberArm).whenInactive(climberArm::stop, climberArm);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
