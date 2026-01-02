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
      Pose2d currentPose = m_drivetrain.getPose();
      
      // Debug: Log positions
      System.out.println(String.format(
          "[NavigateToAlgae] POSITIONS - Robot: (%.3f, %.3f) heading=%.2f° | " +
          "Tag: (%.3f, %.3f) heading=%.2f°",
          currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getRotation().getRadians()),
          tagPose.getX(), tagPose.getY(), Math.toDegrees(tagPose.getRotation().getRadians())));
      
      // Calculate target position (in front of the AprilTag for scoring, accounting for stopping distance)
      // Forward is +X in field coordinates
      Translation2d tagTranslation = tagPose.getTranslation();
      Translation2d targetOffset = new Translation2d(kScoringDistanceMeters, 0); // In front (positive X)
      
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
        // This ensures we maintain the forward offset while stopping short
        finalTargetPosition = offsetPosition.minus(
            directionFromTagToRobot.times(kStoppingDistanceMeters));
      } else {
        // Already close to tag, just use offset position
        finalTargetPosition = tagTranslation.plus(targetOffset);
      }
      
      // Calculate angle to face the tag (for scoring)
      Translation2d toTag = tagTranslation.minus(finalTargetPosition);
      Rotation2d angleToTag = new Rotation2d(Math.atan2(toTag.getY(), toTag.getX()));
      
      // Target pose: in front of tag, facing the tag
      m_targetPose = new Pose2d(finalTargetPosition, angleToTag);
      
      // Debug: Log target position when it's calculated or updated
      System.out.println(String.format(
          "[NavigateToAlgae] TARGET POSE - Position: (%.3f, %.3f) heading=%.2f° | " +
          "Distance to tag: %.3f m | Stopping distance: %.3f m",
          m_targetPose.getX(), m_targetPose.getY(), Math.toDegrees(m_targetPose.getRotation().getRadians()),
          distanceToTag, kStoppingDistanceMeters));
      
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
          
          System.out.println("[NavigateToAlgae] Generated PathPlanner path to target at (" +
              String.format("%.2f", m_targetPose.getX()) + ", " +
              String.format("%.2f", m_targetPose.getY()) + ")");
        } catch (Exception e) {
          System.out.println("[NavigateToAlgae] Error generating path: " + e.getMessage());
          e.printStackTrace();
          m_currentPath = null;
        }
      }
      
      // Follow the path - drive toward target using proper path calculation
      if (m_currentPath != null && m_targetPose != null) {
        // Get vision data for target centering
        double visionYaw = 0.0;
        boolean visionTargetVisible = false;
        try {
          if (m_camera != null) {
            var results = m_camera.getAllUnreadResults();
            if (results != null && !results.isEmpty()) {
              var result = results.get(results.size() - 1);
              if (result != null && result.hasTargets()) {
                var visionTarget = result.getBestTarget();
                if (visionTarget != null && visionTarget.getFiducialId() == targetID) {
                  visionYaw = visionTarget.getYaw(); // Yaw in degrees, negative = left, positive = right
                  visionTargetVisible = true;
                }
              }
            }
          }
        } catch (Exception e) {
          // Vision not available, continue with field-based navigation
        }
        
        // Calculate distance to stopping point (reuse distanceToTag calculated above)
        // distanceToTag is already calculated at line 105 from tagToRobot.getNorm()
        double distanceToStoppingPoint = distanceToTag - kStoppingDistanceMeters;
        
        // Calculate errors to target pose
        double xError = m_targetPose.getX() - currentPose.getX();
        double yError = m_targetPose.getY() - currentPose.getY();
        
        // Calculate angle to target in field coordinates
        // atan2(y, x) gives angle: 0 = +X (forward), +PI/2 = +Y (left), -PI/2 = -Y (right)
        double angleToTarget = Math.atan2(yError, xError);
        double currentHeading = currentPose.getRotation().getRadians();
        
        // Fix for potentially inverted gyro: negate the heading if gyro is inverted
        // This handles the case where the gyro reports angles in the opposite direction
        currentHeading = -currentHeading;
        
        // Calculate heading error (how much we need to turn)
        double headingError = angleToTarget - currentHeading;
        
        // Normalize heading error to [-PI, PI]
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        
        // Calculate distance to target pose
        double distanceToTarget = Math.sqrt(xError * xError + yError * yError);
        
        // Forward speed: drive toward target at maximum speed
        // Allow forward movement even when somewhat misaligned (within 135 degrees)
        // This prevents the robot from getting stuck when it needs to turn
        double forwardSpeed = 0.0;
        if (Math.abs(headingError) < Math.toRadians(135)) {
          // Base speed: maximum speed (1.0), only slow down when extremely close
          double baseSpeed = 1.0; // Maximum speed
          
          // Only slow down when extremely close to the stopping point (within 0.2m)
          if (distanceToStoppingPoint < 0.2) {
            // Scale speed down linearly as we approach stopping distance
            double speedScale = Math.max(0.5, distanceToStoppingPoint / 0.2);
            baseSpeed *= speedScale;
          }
          
          // Alignment penalty - reduce speed when misaligned, but still allow movement
          double alignmentFactor = Math.cos(headingError); // 1.0 when aligned, 0.0 when perpendicular
          // Clamp alignment factor to prevent negative values and maintain reasonable speed
          // When headingError is 135°, cos(135°) ≈ -0.7, so we need to handle this
          alignmentFactor = Math.max(0.3, alignmentFactor); // At least 30% speed even when very misaligned
          
          // Combine: base speed * alignment factor
          forwardSpeed = baseSpeed * alignmentFactor;
          // Ensure reasonable minimum speed when far away
          if (distanceToStoppingPoint > 0.2) {
            // When well aligned (headingError < 45°), go fast. When misaligned, still move but slower
            if (Math.abs(headingError) < Math.toRadians(45)) {
              forwardSpeed = Math.max(0.9, Math.min(1.0, forwardSpeed)); // At least 90% when well aligned
            } else {
              forwardSpeed = Math.max(0.4, Math.min(1.0, forwardSpeed)); // At least 40% when misaligned but within 135°
            }
          } else {
            forwardSpeed = Math.max(0.3, Math.min(1.0, forwardSpeed)); // Allow slower only when extremely close
          }
        } else {
          // Too far off heading, don't drive forward, just turn
          forwardSpeed = 0.0;
        }
        
        // Rotation: combine field-based heading error with vision-based centering
        double rotationSpeed = 0.0;
        
        if (visionTargetVisible && Math.abs(visionYaw) > kVisionYawToleranceDegrees) {
          // Use vision to center target (fine alignment)
          // visionYaw: negative = target left of center, positive = target right of center
          // We want to turn toward the target, so negate yaw
          double visionRotation = -visionYaw * Constants.Vision.kVisionTurnkP;
          rotationSpeed = visionRotation;
        } else {
          // Use field-based navigation (coarse alignment)
          // Note: Negate headingError because positive rotation in WPILib is counterclockwise (left turn)
          // When headingError is negative (need to turn left), we want positive rotation
          rotationSpeed = -headingError * Constants.PathPlanner.kRotationkP;
        }
        
        // Safety check: if heading error is very large (>150°), log a warning
        // This might indicate a coordinate system issue or incorrect pose initialization
        if (Math.abs(headingError) > Math.toRadians(150)) {
          System.out.println(String.format(
              "[NavigateToAlgae] WARNING: Large heading error detected: %.1f° " +
              "(currentHeading=%.1f°, angleToTarget=%.1f°). " +
              "Check robot pose initialization and coordinate system.",
              Math.toDegrees(headingError), Math.toDegrees(currentHeading), Math.toDegrees(angleToTarget)));
        }
        
        // Debug output with detailed position information
        // Get tag position again for logging (it's already calculated above)
        Pose2d tagPoseForLog = Vision.getAprilTagPose(targetID, new Transform2d());
        Translation2d tagPosForLog = tagPoseForLog.getTranslation();
        System.out.println(String.format(
            "[NavigateToAlgae] EXECUTE - Robot: (%.3f, %.3f) heading=%.2f° | " +
            "Target: (%.3f, %.3f) heading=%.2f° | " +
            "Tag: (%.3f, %.3f) | " +
            "distToTag=%.3f distToStop=%.3f xErr=%.3f yErr=%.3f | " +
            "angleToTarget=%.2f° headingErr=%.2f° visionYaw=%.2f° | " +
            "fwd=%.3f rot=%.3f",
            currentPose.getX(), currentPose.getY(), Math.toDegrees(currentHeading),
            m_targetPose.getX(), m_targetPose.getY(), Math.toDegrees(m_targetPose.getRotation().getRadians()),
            tagPosForLog.getX(), tagPosForLog.getY(),
            distanceToTag, distanceToStoppingPoint,
            xError, yError, Math.toDegrees(angleToTarget),
            Math.toDegrees(headingError), visionYaw, forwardSpeed, rotationSpeed));
        
        m_drivetrain.arcadeDrive(forwardSpeed, rotationSpeed);
        
        SmartDashboard.putNumber("NavigateToAlgae/Distance to Tag", distanceToTag);
        SmartDashboard.putNumber("NavigateToAlgae/Distance to Stopping Point", distanceToStoppingPoint);
        SmartDashboard.putNumber("NavigateToAlgae/Distance to Target Pose", distanceToTarget);
        SmartDashboard.putNumber("NavigateToAlgae/X Error", xError);
        SmartDashboard.putNumber("NavigateToAlgae/Y Error", yError);
        SmartDashboard.putNumber("NavigateToAlgae/Heading Error (deg)", Math.toDegrees(headingError));
        SmartDashboard.putNumber("NavigateToAlgae/Angle to Target (deg)", Math.toDegrees(angleToTarget));
        SmartDashboard.putNumber("NavigateToAlgae/Current Heading (deg)", Math.toDegrees(currentHeading));
        SmartDashboard.putNumber("NavigateToAlgae/Vision Yaw (deg)", visionYaw);
        SmartDashboard.putBoolean("NavigateToAlgae/Vision Target Visible", visionTargetVisible);
      } else {
        // No path, stop
        m_drivetrain.arcadeDrive(0.0, 0.0);
      }
    } catch (Exception e) {
      System.out.println("[NavigateToAlgaeScoring] Error: " + e.getMessage());
      e.printStackTrace();
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
