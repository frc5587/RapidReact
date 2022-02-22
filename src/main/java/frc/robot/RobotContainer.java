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

import frc.robot.subsystems.Shooter;

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
  public static final Shooter shooter = new Shooter();

  
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
      // Instantiate buttons on xboxController
      JoystickButton xButton = new JoystickButton(xboxController, XboxController.Button.kX.value);
      // Instantiate triggers on xboxController
      Trigger leftTrigger = new Trigger(() -> xboxController.getLeftTriggerAxis() > 0);


      /*
      Shooter
      */

      /**
      * When the X-button is active, run the motor forwards.
      * When it is not active, stop the motor.
      */
      xButton.and(leftTrigger.negate()).whenActive(shooter::flyWheelForwards, shooter).whenInactive(shooter::stop, shooter);

      /**
      * When the X-button and left trigger are active, run the motor backwards.
      * When they are not active, stop the motor.
      */
      xButton.and(leftTrigger).whenActive(shooter::flyWheelBackwards, shooter).whenInactive(shooter::stop, shooter);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}