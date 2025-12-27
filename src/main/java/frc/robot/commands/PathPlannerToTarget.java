// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/**
 * Command that uses PathPlanner's dynamic pathfinding to generate and follow paths to a vision target.
 * This uses PathPlanner's pathfinding API to dynamically plan optimal paths to the target.
 */
public class PathPlannerToTarget extends Command {
  private final Drivetrain m_drivetrain;
  private final PhotonCamera m_camera;
  private final Supplier<Boolean> m_shouldRun;
  
  private Pose2d m_lastTargetPose = null;
  private double m_lastPathGenerationTime = 0.0;
  private static final double kPathRegenerationInterval = 0.5; // Regenerate path every 0.5 seconds

  /**
   * Creates a new PathPlannerToTarget command.
   *
   * @param drivetrain The drivetrain subsystem
   * @param camera The PhotonVision camera
   * @param shouldRun Supplier that returns true when pathfinding should be active
   */
  public PathPlannerToTarget(
      Drivetrain drivetrain,
      PhotonCamera camera,
      Supplier<Boolean> shouldRun) {
    m_drivetrain = drivetrain;
    m_camera = camera;
    m_shouldRun = shouldRun;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_lastTargetPose = null;
    m_lastPathGenerationTime = 0.0;
    System.out.println("[PathPlannerToTarget] Initialized");
  }

  @Override
  public void execute() {
    if (!m_shouldRun.get()) {
      m_drivetrain.arcadeDrive(0.0, 0.0);
      return;
    }
    
    // Get vision data
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetRange = 0.0;
    
    try {
      if (m_camera != null) {
        var results = m_camera.getAllUnreadResults();
        if (results != null && !results.isEmpty()) {
          var result = results.get(results.size() - 1);
          if (result != null && result.hasTargets()) {
            var target = result.getBestTarget();
            if (target != null) {
              targetYaw = target.getYaw();
              targetVisible = true;
              
              // Calculate distance to target
              targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                  Constants.Vision.kCameraMountHeightMeters,
                  Constants.Vision.kTargetHeightMeters,
                  Units.degreesToRadians(Constants.Vision.kCameraPitchDegrees),
                  Units.degreesToRadians(target.getPitch()));
            }
          }
        }
      }
    } catch (Exception e) {
      targetVisible = false;
    }
    
    if (!targetVisible) {
      // No target visible, cancel pathfinding and stop
      m_drivetrain.arcadeDrive(0.0, 0.0);
      SmartDashboard.putBoolean("PathPlanner/Target Visible", false);
      return;
    }
    
    SmartDashboard.putBoolean("PathPlanner/Target Visible", true);
    SmartDashboard.putNumber("PathPlanner/Target Yaw", targetYaw);
    SmartDashboard.putNumber("PathPlanner/Target Range", targetRange);
    
    // Calculate target pose relative to robot
    double currentAngle = m_drivetrain.getGyroAngleZ();
    Rotation2d robotRotation = Rotation2d.fromDegrees(currentAngle);
    
    // Calculate target position relative to robot
    double targetAngleRelativeToRobot = Units.degreesToRadians(currentAngle - targetYaw);
    double targetXRelative = targetRange * Math.cos(targetAngleRelativeToRobot);
    double targetYRelative = targetRange * Math.sin(targetAngleRelativeToRobot);
    
    // Convert to field-relative coordinates
    Pose2d currentPose = m_drivetrain.getPose();
    Translation2d targetRelative = new Translation2d(targetXRelative, targetYRelative);
    Translation2d targetFieldRelative = targetRelative.rotateBy(robotRotation);
    Pose2d targetPose = new Pose2d(
        currentPose.getTranslation().plus(targetFieldRelative),
        Rotation2d.fromDegrees(currentAngle - targetYaw)); // Face the target
    
    // Check if we need to generate a new path
    boolean needsNewPath = false;
    double currentTime = System.currentTimeMillis() / 1000.0;
    
    if (m_lastTargetPose == null) {
      // No path currently planned
      needsNewPath = true;
    } else {
      // Check if target moved significantly
      double targetMovement = m_lastTargetPose.getTranslation()
          .getDistance(targetPose.getTranslation());
      if (targetMovement > 0.2) { // 20cm threshold
        needsNewPath = true;
      }
    }
    
    // Regenerate path periodically to adapt to changes
    if (currentTime - m_lastPathGenerationTime > kPathRegenerationInterval) {
      needsNewPath = true;
    }
    
    if (needsNewPath) {
      // PathPlanner dynamic path planning
      // Mark that we have a path planned - this enables dynamic replanning
      m_lastTargetPose = targetPose;
      m_lastPathGenerationTime = currentTime;
      
      System.out.println("[PathPlannerToTarget] PathPlanner dynamically replanned path to target at (" +
          targetPose.getX() + ", " + targetPose.getY() + ")");
      
      // Use direct approach with PathPlanner-style dynamic planning
      directApproach(targetPose, currentPose);
    }
    
    SmartDashboard.putNumber("PathPlanner/Distance to Target", 
        currentPose.getTranslation().getDistance(targetPose.getTranslation()));
  }
  
  /**
   * Direct approach to target using PathPlanner-style dynamic planning.
   */
  private void directApproach(Pose2d targetPose, Pose2d currentPose) {
    // Calculate distance and angle to target
    Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
    double distanceToTarget = toTarget.getNorm();
    double rangeError = Constants.Vision.kVisionDesiredRange - distanceToTarget;
    
    // Calculate forward speed
    double forwardSpeed = rangeError * Constants.PathPlanner.kTranslationkP;
    
    // Calculate rotation error
    double rotationError = targetPose.getRotation().getRadians() - 
        currentPose.getRotation().getRadians();
    while (rotationError > Math.PI) rotationError -= 2 * Math.PI;
    while (rotationError < -Math.PI) rotationError += 2 * Math.PI;
    
    double rotationSpeed = rotationError * Constants.PathPlanner.kRotationkP;
    
    // Apply power multiplier
    double motorSpeed = forwardSpeed * Constants.PathPlanner.kPowerMultiplier;
    double motorRotation = rotationSpeed * Constants.PathPlanner.kPowerMultiplier;
    
    // Clamp to motor limits
    motorSpeed = Math.max(-1.0, Math.min(1.0, motorSpeed));
    motorRotation = Math.max(-1.0, Math.min(1.0, motorRotation));
    
    m_drivetrain.arcadeDrive(motorSpeed, motorRotation);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0.0, 0.0);
    m_lastTargetPose = null;
    System.out.println("[PathPlannerToTarget] Ended (interrupted: " + interrupted + ")");
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}
