// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/**
 * Command to navigate to the left side of a target AprilTag.
 */
public class NavigateToLeftOfTarget extends Command {
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  
  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;
  
  private Pose2d m_targetPose = null;
  private static final double kOffsetDistanceMeters = 0.5; // 50cm to the left of target
  
  /**
   * Creates a new NavigateToLeftOfTarget command.
   *
   * @param drivetrain The drivetrain subsystem
   * @param vision The vision subsystem
   */
  public NavigateToLeftOfTarget(Drivetrain drivetrain, Vision vision) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    addRequirements(drivetrain);
    
    // Initialize PID controllers
    m_xController = new PIDController(
        Constants.PathPlanner.kTranslationkP,
        Constants.PathPlanner.kTranslationkI,
        Constants.PathPlanner.kTranslationkD);
    m_yController = new PIDController(
        Constants.PathPlanner.kTranslationkP,
        Constants.PathPlanner.kTranslationkI,
        Constants.PathPlanner.kTranslationkD);
    m_rotationController = new PIDController(
        Constants.PathPlanner.kRotationkP,
        Constants.PathPlanner.kRotationkI,
        Constants.PathPlanner.kRotationkD);
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    System.out.println("[NavigateToLeftOfTarget] Initialized");
    m_targetPose = null;
  }

  @Override
  public void execute() {
    // Hard-coded to tag 1 for testing
    int targetID = 1;
    
    SmartDashboard.putBoolean("NavigateToLeft/Target Found", true);
    SmartDashboard.putNumber("NavigateToLeft/Target ID", targetID);
    
      try {
      Pose2d tagPose = Vision.getAprilTagPose(targetID, new Transform2d());
      Pose2d currentPose = m_drivetrain.getPose();
      
      // Calculate target position (to the left of the AprilTag)
      // Left side is +Y in field coordinates (forward is +X)
      Translation2d targetTranslation = tagPose.getTranslation().plus(
          new Translation2d(0, kOffsetDistanceMeters)); // Left side (positive Y)
      
      // Calculate desired heading - face toward the target position
      Translation2d toTarget = targetTranslation.minus(currentPose.getTranslation());
      Rotation2d desiredHeading = new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
      
      // Create target pose with desired heading
      m_targetPose = new Pose2d(targetTranslation, desiredHeading);
      
      // Calculate error
      Translation2d error = m_targetPose.getTranslation().minus(currentPose.getTranslation());
      
      // Calculate speeds
      double xSpeed = m_xController.calculate(currentPose.getX(), m_targetPose.getX());
      double ySpeed = m_yController.calculate(currentPose.getY(), m_targetPose.getY());
      
      // Convert to robot-relative speeds
      Rotation2d robotAngle = currentPose.getRotation();
      double speed = Math.cos(robotAngle.getRadians()) * xSpeed + Math.sin(robotAngle.getRadians()) * ySpeed;
      
      // Rotation: face toward target, but with lower priority (scale down rotation gain)
      double rotation = m_rotationController.calculate(
          currentPose.getRotation().getRadians(),
          m_targetPose.getRotation().getRadians()) * 0.5; // Reduce rotation priority
      
      // Apply power multiplier and clamp
      speed *= Constants.PathPlanner.kPowerMultiplier;
      rotation *= Constants.PathPlanner.kPowerMultiplier;
      speed = Math.max(-1.0, Math.min(1.0, speed));
      rotation = Math.max(-1.0, Math.min(1.0, rotation));
      
      m_drivetrain.arcadeDrive(speed, rotation);
      
      SmartDashboard.putNumber("NavigateToLeft/Distance to Target", error.getNorm());
      SmartDashboard.putNumber("NavigateToLeft/X Error", error.getX());
      SmartDashboard.putNumber("NavigateToLeft/Y Error", error.getY());
    } catch (Exception e) {
      System.out.println("[NavigateToLeftOfTarget] Error calculating pose: " + e.getMessage());
      m_drivetrain.arcadeDrive(0.0, 0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0.0, 0.0);
    System.out.println("[NavigateToLeftOfTarget] Ended (interrupted: " + interrupted + ")");
  }

  @Override
  public boolean isFinished() {
    if (m_targetPose == null) {
      return false;
    }
    
    Pose2d currentPose = m_drivetrain.getPose();
    double distance = currentPose.getTranslation().getDistance(m_targetPose.getTranslation());
    double angleError = Math.abs(currentPose.getRotation().minus(m_targetPose.getRotation()).getRadians());
    
    // Finished when close enough to target
    return distance < 0.15 && angleError < Units.degreesToRadians(5.0);
  }
}


