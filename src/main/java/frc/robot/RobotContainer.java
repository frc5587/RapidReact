// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandXboxController;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;

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
  private final Conveyor conveyor = new Conveyor();
  private final Kicker rightKicker = Kicker.createRightKicker();
  private final Kicker leftKicker = Kicker.createLeftKicker();
  private final Intake intake = new Intake();
  private final IntakePistons intakePistons = new IntakePistons();

  // Others

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Instantiate controller bindings
    JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
    JoystickButton xButton = new JoystickButton(xboxController, XboxController.Button.kX.value);
    Trigger leftTrigger = new Trigger(() -> xboxController.getLeftTriggerAxis() > 0);    

    /**
     * INTAKE
     */
    aButton.and(leftTrigger.negate())
        .whileActiveOnce(new IntakeIn(intake, intakePistons, conveyor));
        // .whenInactive(new RunKickerUp(conveyor, rightKicker, leftKicker));
    aButton.and(leftTrigger)
        .whileActiveOnce(new IntakeOut(intake, intakePistons, conveyor));
    
    /**
     * Kicker
     */

    xButton.and(leftTrigger.negate())
      .whileActiveOnce(new RunKickerUp(conveyor, rightKicker, leftKicker));
      
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