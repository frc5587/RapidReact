// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

import org.frc5587.lib.control.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final DeadbandJoystick joystick = new DeadbandJoystick(0, 1.5);
  private final DeadbandXboxController xboxController = new DeadbandXboxController(1);
  
  // Subsystems
  private final Intake intake = new Intake();
  private final IntakePistons intakePistons = new IntakePistons();
  private final Conveyor conveyor = new Conveyor();
  private final Kicker rightKicker = Kicker.createRightKicker();
  private final Kicker leftKicker = Kicker.createLeftKicker();
  private final LinebreakSensor linebreakSensor = new LinebreakSensor();
  private final Shooter shooter = new Shooter();

  // Commands
  private final Index index = new Index(intake, intakePistons, conveyor, rightKicker, leftKicker, linebreakSensor);
  private final TopBallOut topBallOut = new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter);
  private final BottomBallOut bottomBallOut = new BottomBallOut(intake, intakePistons, conveyor);
  private final ShootDashboard shootDashboard = new ShootDashboard(shooter, rightKicker, leftKicker, shooter::getSmartDashboard);
  private final ShootOne shootOne = new ShootOne(conveyor, rightKicker, leftKicker, linebreakSensor, shooter, shooter::getSmartDashboard);

  private final MoveDown moveDown = new MoveDown(intake, intakePistons, conveyor, rightKicker, leftKicker, linebreakSensor);
  private final IntakeOnly intakeOnly = new IntakeOnly(intake, intakePistons);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Driver Station configuration
    DriverStation.silenceJoystickConnectionWarning(true);
    // Configure the button bindings
    configureButtonBindings();
    // Configure the default commands
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
    /**
     * Instantiate controller bindings
     */

    // Xbox Controller buttons
    JoystickButton xButton = new JoystickButton(xboxController, XboxController.Button.kX.value);
    JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
    JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);
    JoystickButton yButton = new JoystickButton(xboxController, XboxController.Button.kY.value);

    // Xbox Controller POV buttons
    POVButton dpadUp = new POVButton(xboxController, 0);
    POVButton dpadDown = new POVButton(xboxController, 180);

    // Xbox Controller triggers
    Trigger leftTrigger = new Trigger(() -> xboxController.getLeftTrigger());
    Trigger rightTrigger = new Trigger(() -> xboxController.getLeftTrigger());

    // Xbox Controller sticks
    Trigger leftStickY = new Trigger(() -> { 
      return xboxController.getLeftY() != 0;
    });
    Trigger rightStickY = new Trigger(() -> {
      return xboxController.getLeftY() != 0;
    });

    
    /**
     * INTAKE
     */
    aButton.and(leftTrigger.negate())
      .whileActiveOnce(index);

    // yButton.and(leftTrigger.negate())
    //   .whileActiveOnce(moveDown);

    bButton.and(leftTrigger.negate())
      .whileActiveOnce(intakeOnly);
    dpadUp
      .whenHeld(new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter));
      
    dpadDown
      .whenHeld(bottomBallOut);


    /**
     * SHOOTER
     */
    // xButton
    //   .whenHeld(shootDashboard);
    xButton
      .whenHeld(shootOne);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}