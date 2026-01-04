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
import org.photonvision.PhotonCamera;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.RotationTarget;

/**
 * Command to navigate to algae scoring position using the best reef target.
 */
public class NavigateToAlgaeScoring extends Command {
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  private final PhotonCamera m_camera;
  
  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;
  
  private Pose2d m_targetPose = null;
  private PathPlannerPath m_currentPath = null;
  private double m_pathStartTime = 0.0;
  private double m_lastPathGenerationTime = 0.0;
  private static final double kScoringDistanceMeters = 0.5; // 50cm in front of target for scoring
  private static final double kStoppingDistanceMeters = Units.inchesToMeters(6.0); // Stop 6" before target
  private static final double kPathRegenerationInterval = 0.5; // Regenerate path every 0.5 seconds
  private static final double kVisionYawToleranceDegrees = 2.0; // Stop rotating when target is within 2° of center
  private static final double kSlowDownDistanceMeters = 0.5; // Start slowing down 0.5m from stopping point
  
  /**
   * Creates a new NavigateToAlgaeScoring command.
   *
   * @param drivetrain The drivetrain subsystem
   * @param vision The vision subsystem
   */
  public NavigateToAlgaeScoring(Drivetrain drivetrain, Vision vision) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_camera = new PhotonCamera(Constants.Vision.kCameraName);
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
    System.out.println("[NavigateToAlgaeScoring] Initialized");
    m_targetPose = null;
    m_currentPath = null;
    m_pathStartTime = 0.0;
    m_lastPathGenerationTime = 0.0;
    
    // Get tag and generate path once at initialization
    int targetID = 1;
    try {
      Pose2d tagPose = Vision.getAprilTagPose(targetID, new Transform2d());
      Pose2d currentPose = m_drivetrain.getPose();
      
      // Calculate target position (in front of the AprilTag for scoring)
      Translation2d tagTranslation = tagPose.getTranslation();
      Translation2d targetOffset = new Translation2d(kScoringDistanceMeters, 0); // In front (positive X)
      
      // Calculate final target position
      Translation2d tagToRobot = currentPose.getTranslation().minus(tagTranslation);
      double distanceToTag = tagToRobot.getNorm();
      Translation2d finalTargetPosition;
      if (distanceToTag > kStoppingDistanceMeters) {
        Translation2d offsetPosition = tagTranslation.plus(targetOffset);
        Translation2d directionFromTagToRobot = tagToRobot.div(distanceToTag);
        finalTargetPosition = offsetPosition.minus(
            directionFromTagToRobot.times(kStoppingDistanceMeters));
      } else {
        finalTargetPosition = tagTranslation.plus(targetOffset);
      }
      
      // Calculate angle to face the tag (for scoring)
      Translation2d toTag = tagTranslation.minus(finalTargetPosition);
      Rotation2d angleToTag = new Rotation2d(Math.atan2(toTag.getY(), toTag.getX()));
      
      // Target pose: in front of tag, facing the tag
      m_targetPose = new Pose2d(finalTargetPosition, angleToTag);
      
      // Generate path once
      PathConstraints constraints = new PathConstraints(
          Constants.PathPlanner.kMaxSpeedMetersPerSecond,
          Constants.PathPlanner.kMaxAccelerationMetersPerSecondSquared,
          Constants.PathPlanner.kMaxAngularSpeedRadiansPerSecond,
          Constants.PathPlanner.kMaxAngularAccelerationRadiansPerSecondSquared);
      
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
      
      m_pathStartTime = System.currentTimeMillis() / 1000.0;
      
      System.out.println("[NavigateToAlgae] Generated PathPlanner path from (" +
          String.format("%.2f", currentPose.getX()) + ", " +
          String.format("%.2f", currentPose.getY()) + ") to (" +
          String.format("%.2f", m_targetPose.getX()) + ", " +
          String.format("%.2f", m_targetPose.getY()) + ")");
    } catch (Exception e) {
      System.out.println("[NavigateToAlgae] Error initializing path: " + e.getMessage());
      e.printStackTrace();
    }
  }

  @Override
  public void execute() {
    // Follow the PathPlanner path - drive from point A to point B
    if (m_currentPath != null && m_targetPose != null) {
      Pose2d currentPose = m_drivetrain.getPose();
      
      // Get all poses along the path
      var pathPoses = m_currentPath.getPathPoses();
      if (pathPoses != null && !pathPoses.isEmpty()) {
        // Find the closest pose on the path to current position
        double minDistance = Double.MAX_VALUE;
        int closestIndex = 0;
        
        for (int i = 0; i < pathPoses.size(); i++) {
          double distance = pathPoses.get(i).getTranslation()
              .getDistance(currentPose.getTranslation());
          if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
          }
        }
        
        // Look ahead along the path (use next pose if available)
        int lookaheadIndex = Math.min(closestIndex + 3, pathPoses.size() - 1);
        Pose2d desiredPose = pathPoses.get(lookaheadIndex);
        
        // Calculate direction to desired pose
        double xError = desiredPose.getX() - currentPose.getX();
        double yError = desiredPose.getY() - currentPose.getY();
        double distanceToDesired = Math.sqrt(xError * xError + yError * yError);
        
        // Forward speed: use PID for minor error correction
        // Calculate forward component based on heading alignment
        double angleToDesired = Math.atan2(yError, xError);
        double currentHeading = currentPose.getRotation().getRadians();
        currentHeading = -currentHeading; // Fix for inverted gyro
        double headingError = angleToDesired - currentHeading;
        
        // Normalize heading error
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        
        // Forward speed: always set to 5
        double forwardSpeed = 5.0;
        
        // Rotation: use PID for minor error correction
        double desiredHeading = desiredPose.getRotation().getRadians();
        double rotationError = desiredHeading - currentHeading;
        
        // Normalize rotation error to [-PI, PI]
        while (rotationError > Math.PI) rotationError -= 2 * Math.PI;
        while (rotationError < -Math.PI) rotationError += 2 * Math.PI;
        
        // Minor error correction with PID (reduced gain to prevent wobbling)
        double rotationSpeed = 0.0;
        double rotationErrorDegrees = Math.toDegrees(rotationError);
        
        if (Math.abs(rotationErrorDegrees) > 3.0) { // 3 degree deadband
          rotationSpeed = m_rotationController.calculate(currentHeading, desiredHeading);
          // Reduce gain for smoother control
          rotationSpeed *= 0.3; // Reduce by 70% for minor correction
          rotationSpeed = Math.max(-0.6, Math.min(0.6, rotationSpeed)); // Limit speed
        } else {
          // Close enough, stop rotating
          rotationSpeed = 0.0;
          m_rotationController.reset();
        }
        
        // Drive the path - following the dynamic path from PathPlanner
        m_drivetrain.arcadeDrive(forwardSpeed, rotationSpeed);
        
        // Debug output
        System.out.println(String.format(
            "[NavigateToAlgae] fwd=%.2f rot=%.2f dist=%.2f headingErr=%.1f° pathPoses=%d",
            forwardSpeed, rotationSpeed, distanceToDesired, Math.toDegrees(headingError), pathPoses.size()));
        
        SmartDashboard.putNumber("NavigateToAlgae/Distance to Desired", distanceToDesired);
        SmartDashboard.putNumber("NavigateToAlgae/Closest Index", closestIndex);
        SmartDashboard.putNumber("NavigateToAlgae/Rotation Error (deg)", rotationErrorDegrees);
        SmartDashboard.putNumber("NavigateToAlgae/Forward Speed", forwardSpeed);
        SmartDashboard.putNumber("NavigateToAlgae/Heading Error (deg)", Math.toDegrees(headingError));
      } else {
        // No path poses available, stop
        m_drivetrain.arcadeDrive(0.0, 0.0);
      }
    } else {
      // No path, stop
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
    
    try {
      Pose2d currentPose = m_drivetrain.getPose();
      
      // Get tag pose to calculate distance to actual tag
      int targetID = 1; // Hard-coded for now
      Pose2d tagPose = Vision.getAprilTagPose(targetID, new Transform2d());
      Translation2d tagTranslation = tagPose.getTranslation();
      
      // Calculate distance to actual tag (not target pose)
      double distanceToTag = currentPose.getTranslation().getDistance(tagTranslation);
      
      // Check if we're within stopping distance of the tag
      boolean withinStoppingDistance = distanceToTag <= kStoppingDistanceMeters;
      
      // Check if target is centered in vision (if visible)
      boolean targetCentered = true;
      try {
        if (m_camera != null) {
          var results = m_camera.getAllUnreadResults();
          if (results != null && !results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result != null && result.hasTargets()) {
              var visionTarget = result.getBestTarget();
              if (visionTarget != null && visionTarget.getFiducialId() == targetID) {
                double visionYaw = Math.abs(visionTarget.getYaw());
                targetCentered = visionYaw <= kVisionYawToleranceDegrees;
              }
            }
          }
        }
      } catch (Exception e) {
        // Vision not available, skip centering check
      }
      
      // Finished when: within stopping distance AND target is centered
      return withinStoppingDistance && targetCentered;
    } catch (Exception e) {
      return false;
    }
  }
}
