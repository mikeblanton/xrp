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
 * Command to navigate to algae scoring position using the best reef target.
 */
public class NavigateToAlgaeScoring extends Command {
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  
  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;
  
  private Pose2d m_targetPose = null;
  private static final double kScoringDistanceMeters = 0.5; // 50cm in front of target for scoring
  
  /**
   * Creates a new NavigateToAlgaeScoring command.
   *
   * @param drivetrain The drivetrain subsystem
   * @param vision The vision subsystem
   */
  public NavigateToAlgaeScoring(Drivetrain drivetrain, Vision vision) {
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
    System.out.println("[NavigateToAlgaeScoring] Initialized");
    m_targetPose = null;
  }

  @Override
  public void execute() {
    // Hard-coded to tag 1 for testing
    // For competition, change this to use m_vision.getBestReefTarget() to only use alliance-specific tags
    int targetID = 1;
    
    SmartDashboard.putBoolean("NavigateToAlgae/Target Found", true);
    SmartDashboard.putNumber("NavigateToAlgae/Target ID", targetID);
    
    try {
      Pose2d tagPose = Vision.getAprilTagPose(targetID, new Transform2d());
      
      // Calculate angle to face the target
      Pose2d currentPose = m_drivetrain.getPose();
      Translation2d toTag = tagPose.getTranslation().minus(currentPose.getTranslation());
      Rotation2d angleToTarget = new Rotation2d(Math.atan2(toTag.getY(), toTag.getX()));
      
      // Create offset that faces the target (in front of target for scoring)
      // Forward is +X in field coordinates
      Transform2d scoringOffset = new Transform2d(
          new Translation2d(kScoringDistanceMeters, 0),
          angleToTarget.minus(Rotation2d.fromDegrees(180))); // Face towards target
      
      m_targetPose = tagPose.transformBy(scoringOffset);
      
      // Navigate to the target pose
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
      
      SmartDashboard.putNumber("NavigateToAlgae/Distance to Target", error.getNorm());
      SmartDashboard.putNumber("NavigateToAlgae/X Error", error.getX());
      SmartDashboard.putNumber("NavigateToAlgae/Y Error", error.getY());
    } catch (Exception e) {
      System.out.println("[NavigateToAlgaeScoring] Error calculating pose: " + e.getMessage());
      m_drivetrain.arcadeDrive(0.0, 0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0.0, 0.0);
    System.out.println("[NavigateToAlgaeScoring] Ended (interrupted: " + interrupted + ")");
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

