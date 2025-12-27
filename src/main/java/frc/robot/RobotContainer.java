// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.Constants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.DriveForward;
import frc.robot.commands.NavigateToAlgaeScoring;
import frc.robot.commands.NavigateToLeftOfTarget;
import frc.robot.commands.NavigateToRightOfTarget;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.photonvision.PhotonCamera;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;

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

  // PhotonVision camera
  // Note: You may see errors/warnings in the console if PhotonVision isn't connected yet.
  // These are expected and won't prevent the robot from driving normally.
  // The errors will disappear once PhotonVision is properly connected and running.
  private final PhotonCamera m_camera = new PhotonCamera(Constants.Vision.kCameraName);

  // Vision subsystem for AprilTag detection and pose estimation
  private final Vision m_vision = new Vision(m_drivetrain::getPose, m_drivetrain.getField());

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure PathPlanner AutoBuilder for differential drive
    configurePathPlanner();
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Configure PathPlanner AutoBuilder for differential drive pathfinding.
   * Note: AutoBuilder configuration may vary by PathPlanner version.
   * If this fails, pathfinding will use a fallback approach.
   */
  private void configurePathPlanner() {
    // PathPlanner AutoBuilder configuration
    // For now, we'll use PathPlanner's pathfinding API directly in commands
    // AutoBuilder can be configured here if needed for specific PathPlanner versions
    System.out.println("[RobotContainer] PathPlanner will be used for dynamic pathfinding");
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

    // Button mappings for vision navigation:
    // Square: Navigate to left of target
    Trigger squareButton = new Trigger(m_controller::getSquareButton);
    squareButton.whileTrue(new NavigateToLeftOfTarget(m_drivetrain, m_vision));

    // Circle: Navigate to right of target
    Trigger circleButton = new Trigger(m_controller::getCircleButton);
    circleButton.whileTrue(new NavigateToRightOfTarget(m_drivetrain, m_vision));

    // X: Navigate to algae scoring position
    Trigger xButton = new Trigger(m_controller::getCrossButton);
    xButton.whileTrue(new NavigateToAlgaeScoring(m_drivetrain, m_vision));

    // Triangle: Point at target (handled in ArcadeDrive command - leave as is)

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
        },
        m_camera,
        m_controller);
  }

  /**
   * Get the Vision subsystem.
   * 
   * @return The Vision subsystem
   */
  public Vision getVision() {
    return m_vision;
  }

  /**
   * Get the Drivetrain subsystem.
   * 
   * @return The Drivetrain subsystem
   */
  public Drivetrain getDrivetrain() {
    return m_drivetrain;
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
