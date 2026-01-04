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
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.RotationTarget;

/**
 * Command to navigate to the right side of a target AprilTag.
 */
public class NavigateToRightOfTarget extends Command {
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  
  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;
  
  private Pose2d m_targetPose = null;
  private PathPlannerPath m_currentPath = null;
  private double m_pathStartTime = 0.0;
  private double m_lastPathGenerationTime = 0.0;
  private static final double kOffsetDistanceMeters = 0.5; // 50cm to the right of target
  private static final double kStoppingDistanceMeters = Units.inchesToMeters(6.0); // Stop 6" before target
  private static final double kPathRegenerationInterval = 0.5; // Regenerate path every 0.5 seconds
  
  /**
   * Creates a new NavigateToRightOfTarget command.
   *
   * @param drivetrain The drivetrain subsystem
   * @param vision The vision subsystem
   */
  public NavigateToRightOfTarget(Drivetrain drivetrain, Vision vision) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    addRequirements(drivetrain);
    
    // Initialize PID controllers for path following
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
    System.out.println("[NavigateToRightOfTarget] Initialized");
    m_targetPose = null;
    m_currentPath = null;
    m_pathStartTime = 0.0;
    m_lastPathGenerationTime = 0.0;
  }

  @Override
  public void execute() {
    // Hard-coded to tag 1 for testing
    int targetID = 1;
    
    SmartDashboard.putBoolean("NavigateToRight/Target Found", true);
    SmartDashboard.putNumber("NavigateToRight/Target ID", targetID);
    
    try {
      Pose2d tagPose = Vision.getAprilTagPose(targetID, new Transform2d());
      Pose2d currentPose = m_drivetrain.getPose();
      
      // Calculate target position (to the right of the AprilTag, accounting for stopping distance)
      // Right side is -Y in field coordinates (forward is +X)
      Translation2d tagTranslation = tagPose.getTranslation();
      Translation2d targetOffset = new Translation2d(0, -kOffsetDistanceMeters); // Right side (negative Y)
      
      // Calculate the direction from tag to robot (to know how to back off)
      Translation2d tagToRobot = currentPose.getTranslation().minus(tagTranslation);
      double distanceToTag = tagToRobot.getNorm();
      
      // Calculate final target position: offset from tag, then back off by stopping distance
      Translation2d finalTargetPosition;
      if (distanceToTag > kStoppingDistanceMeters) {
        // Start with tag position + offset (where we want to be relative to tag)
        Translation2d offsetPosition = tagTranslation.plus(targetOffset);
        
        // Calculate direction from tag toward robot (for backing off)
        Translation2d directionFromTagToRobot = tagToRobot.div(distanceToTag);
        
        // Move back from tag along the tag->robot direction by stopping distance
        // This ensures we maintain the right offset while stopping short
        finalTargetPosition = offsetPosition.minus(
            directionFromTagToRobot.times(kStoppingDistanceMeters));
      } else {
        // Already close to tag, just use offset position
        finalTargetPosition = tagTranslation.plus(targetOffset);
      }
      
      // Target pose: horizontally aligned (face forward)
      m_targetPose = new Pose2d(finalTargetPosition, Rotation2d.fromDegrees(0));
      
      // Check if we need to generate a new path
      boolean needsNewPath = false;
      double currentTime = System.currentTimeMillis() / 1000.0;
      
      if (m_currentPath == null || m_targetPose == null) {
        needsNewPath = true;
      } else {
        // Check if target moved significantly
        double targetMovement = m_targetPose.getTranslation()
            .getDistance(tagPose.getTranslation().plus(targetOffset));
        if (targetMovement > 0.2) { // 20cm threshold
          needsNewPath = true;
        }
        
        // Regenerate path periodically
        if (currentTime - m_lastPathGenerationTime > kPathRegenerationInterval) {
          needsNewPath = true;
        }
      }
      
      // Generate new path using PathPlanner
      if (needsNewPath && m_targetPose != null) {
        try {
          PathConstraints constraints = new PathConstraints(
              Constants.PathPlanner.kMaxSpeedMetersPerSecond,
              Constants.PathPlanner.kMaxAccelerationMetersPerSecondSquared,
              Constants.PathPlanner.kMaxAngularSpeedRadiansPerSecond,
              Constants.PathPlanner.kMaxAngularAccelerationRadiansPerSecondSquared);
          
          // Create PathPlanner path from current pose to target pose
          m_currentPath = PathPlannerPath.fromPathPoints(
              java.util.List.of(
                  new PathPoint(
                      currentPose.getTranslation(),
                      new RotationTarget(1.0, currentPose.getRotation())),
                  new PathPoint(
                      m_targetPose.getTranslation(),
                      new RotationTarget(1.0, m_targetPose.getRotation()))
              ),
              constraints,
              new GoalEndState(0.0, m_targetPose.getRotation()));
          
          m_pathStartTime = currentTime;
          m_lastPathGenerationTime = currentTime;
          
          System.out.println("[NavigateToRight] Generated PathPlanner path to target at (" +
              String.format("%.2f", m_targetPose.getX()) + ", " +
              String.format("%.2f", m_targetPose.getY()) + ")");
        } catch (Exception e) {
          System.out.println("[NavigateToRight] Error generating path: " + e.getMessage());
          e.printStackTrace();
          m_currentPath = null;
        }
      }
      
      // Follow the path using smooth path following
      if (m_currentPath != null && m_targetPose != null) {
        // Calculate errors to target pose
        double xError = m_targetPose.getX() - currentPose.getX();
        double yError = m_targetPose.getY() - currentPose.getY();
        double distanceToTarget = Math.sqrt(xError * xError + yError * yError);
        
        // Calculate angle to target in field coordinates
        double angleToTarget = Math.atan2(yError, xError);
        double currentHeading = currentPose.getRotation().getRadians();
        
        // Calculate heading error (how much we need to turn)
        double headingError = angleToTarget - currentHeading;
        
        // Normalize heading error to [-PI, PI]
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        
        // Forward speed: drive toward target with smooth deceleration
        // Use PID controllers for smoother control
        double forwardSpeed = 0.0;
        if (Math.abs(headingError) < Math.PI / 2) {
          // Drive forward, using PID for smooth control
          // Calculate forward component based on heading alignment
          double forwardError = Math.cos(headingError) * distanceToTarget;
          forwardSpeed = m_xController.calculate(0.0, forwardError);
          
          // Scale down as we approach target for smooth stopping
          double speedScale = Math.min(1.0, distanceToTarget / 0.5); // Scale down within 50cm
          forwardSpeed *= speedScale;
          
          // Clamp forward speed
          forwardSpeed = Math.max(-0.8, Math.min(0.8, forwardSpeed));
        } else {
          // Too far off heading, don't drive forward, just turn
          forwardSpeed = 0.0;
        }
        
        // Rotation: turn toward target using PID with reduced gain for smoother control
        double rotationSpeed = 0.0;
        double headingErrorDegrees = Math.toDegrees(headingError);
        
        if (Math.abs(headingErrorDegrees) > 2.0) { // Only rotate if error > 2 degrees
          rotationSpeed = m_rotationController.calculate(currentHeading, angleToTarget);
          
          // Reduce rotation speed for smoother control (prevent wobbling)
          rotationSpeed *= 0.5; // Reduce by 50% for smoother movement
          
          // Clamp rotation speed to prevent overshoot
          rotationSpeed = Math.max(-0.6, Math.min(0.6, rotationSpeed));
        } else {
          // Close enough, stop rotating to prevent oscillation
          rotationSpeed = 0.0;
          m_rotationController.reset(); // Reset PID to prevent integral windup
        }
        
        // Debug output
        System.out.println(String.format(
            "[NavigateToRight] robot=(%.2f,%.2f,%.1f°) target=(%.2f,%.2f) " +
            "xErr=%.2f yErr=%.2f angleToTarget=%.1f° headingErr=%.1f° fwd=%.2f rot=%.2f",
            currentPose.getX(), currentPose.getY(), Math.toDegrees(currentHeading),
            m_targetPose.getX(), m_targetPose.getY(),
            xError, yError, Math.toDegrees(angleToTarget),
            Math.toDegrees(headingError), forwardSpeed, rotationSpeed));
        
        m_drivetrain.arcadeDrive(forwardSpeed, rotationSpeed);
        
        SmartDashboard.putNumber("NavigateToRight/Distance to Target", distanceToTarget);
        SmartDashboard.putNumber("NavigateToRight/X Error", xError);
        SmartDashboard.putNumber("NavigateToRight/Y Error", yError);
        SmartDashboard.putNumber("NavigateToRight/Heading Error (deg)", Math.toDegrees(headingError));
        SmartDashboard.putNumber("NavigateToRight/Angle to Target (deg)", Math.toDegrees(angleToTarget));
        SmartDashboard.putNumber("NavigateToRight/Current Heading (deg)", Math.toDegrees(currentHeading));
      } else {
        // No path, stop
        m_drivetrain.arcadeDrive(0.0, 0.0);
      }
    } catch (Exception e) {
      System.out.println("[NavigateToRightOfTarget] Error: " + e.getMessage());
      e.printStackTrace();
      m_drivetrain.arcadeDrive(0.0, 0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0.0, 0.0);
    System.out.println("[NavigateToRightOfTarget] Ended (interrupted: " + interrupted + ")");
  }

  @Override
  public boolean isFinished() {
    if (m_targetPose == null) {
      return false;
    }
    
    Pose2d currentPose = m_drivetrain.getPose();
    double distance = currentPose.getTranslation().getDistance(m_targetPose.getTranslation());
    double xError = Math.abs(currentPose.getX() - m_targetPose.getX());
    
    // Finished when: within stopping distance AND horizontally aligned
    return distance < kStoppingDistanceMeters && xError < 0.05; // Within 5cm horizontally
  }
}


