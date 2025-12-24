// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.DriveForward;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XRPOnBoardIO m_onboardIO = new XRPOnBoardIO();
  private final Arm m_arm = new Arm();

  // PS5 controller plugged into channel 0
  private final PS5Controller m_controller = new PS5Controller(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.PS5Controller}), and then using {@link Trigger} with the controller's
   * button methods.
   */
  private void configureButtonBindings() {
    // Set default command to arcade drive using left joystick for both X and Y control
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Trigger userButton = new Trigger(m_onboardIO::getUserButtonPressed);
    userButton
        .onTrue(new PrintCommand("USER Button Pressed"))
        .onFalse(new PrintCommand("USER Button Released"));

    Trigger squareButton = new Trigger(m_controller::getSquareButton);
    squareButton
        .onTrue(new InstantCommand(() -> m_arm.setAngle(45.0), m_arm))
        .onFalse(new InstantCommand(() -> m_arm.setAngle(0.0), m_arm));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Apply deadband to controller input to ignore small values near zero.
   * 
   * @param value The input value
   * @param deadband The deadband threshold
   * @return The value with deadband applied, or 0 if below threshold
   */
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return value;
  }

  /**
   * Square the input value for smoother control at low speeds.
   * Preserves the sign of the input.
   * 
   * @param value The input value (-1.0 to 1.0)
   * @return The squared value with sign preserved
   */
  private double squareInput(double value) {
    return Math.copySign(value * value, value);
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   * Uses left joystick for both forward/backward (Y-axis) and left/right (X-axis) control.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, 
        () -> {
          // Left stick Y-axis for forward/backward speed
          double speed = -m_controller.getLeftY();
          speed = applyDeadband(speed, 0.1);
          speed = squareInput(speed);
          return speed;
        }, 
        () -> {
          // Left stick X-axis for left/right rotation
          double rotation = -m_controller.getLeftX();
          rotation = applyDeadband(rotation, 0.1);
          rotation = squareInput(rotation);
          return rotation;
        });
  }

  /**
   * Creates a command that drives the robot forward continuously.
   * 
   * @return a command that drives forward at a constant speed
   */
  public Command getDriveForwardCommand() {
    // Drive forward at speed 2 when X button is held
    // Note: Speed 2.0 is outside normal range (-1.0 to 1.0) but using as requested
    return new DriveForward(2.0, m_drivetrain);
  }
}
