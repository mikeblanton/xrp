// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.RotationTarget;

public class ArcadeDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;
  private final PhotonCamera m_camera;
  private final PS5Controller m_controller;
  
  // Track previous triangle button state to detect presses/releases
  private boolean m_previousTriangleButtonState = false;
  
  // Track previous X button state to detect presses/releases
  private boolean m_previousXButtonState = false;
  
  // Track target angle for smooth rotation (triangle button)
  private Double m_targetAngle = null; // null means not actively aligning
  private boolean m_isAligning = false;
  private double m_previousAngleError = 0.0; // For derivative term calculation
  
  // Smoothing: track previous command values for rate limiting
  private double m_previousRotationCommand = 0.0;
  private double m_previousSpeedCommand = 0.0;
  
  // PathPlanner controllers for improved path following
  private PIDController m_pathXController = null;
  private PIDController m_pathYController = null;
  private PIDController m_pathRotationController = null;
  
  // PathPlanner path from current pose to target pose
  private PathPlannerPath m_currentPath = null;
  private Pose2d m_lastTargetPose = null;
  private double m_lastPathGenerationTime = 0.0;
  private static final double kPathRegenerationInterval = 0.3; // Regenerate path every 0.3 seconds

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   * @param camera The PhotonVision camera for target tracking
   * @param controller The PS5 controller for button input
   */
  public ArcadeDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier,
      PhotonCamera camera,
      PS5Controller controller) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
    m_camera = camera;
    m_controller = controller;
    addRequirements(drivetrain);
    
    // Initialize PathPlanner controllers
    m_pathXController = new PIDController(
        Constants.PathPlanner.kTranslationkP,
        Constants.PathPlanner.kTranslationkI,
        Constants.PathPlanner.kTranslationkD);
    m_pathYController = new PIDController(
        Constants.PathPlanner.kTranslationkP,
        Constants.PathPlanner.kTranslationkI,
        Constants.PathPlanner.kTranslationkD);
    m_pathRotationController = new PIDController(
        Constants.PathPlanner.kRotationkP,
        Constants.PathPlanner.kRotationkI,
        Constants.PathPlanner.kRotationkD);
    m_pathRotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_xaxisSpeedSupplier.get();
    double rotation = m_zaxisRotateSupplier.get();
    
    // Read in relevant data from the Camera
    // Wrap in try-catch so robot can still drive if PhotonVision isn't connected
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetRange = 0.0;
    try {
      if (m_camera != null) {
        var results = m_camera.getAllUnreadResults();
        if (results != null && !results.isEmpty()) {
          // Camera processed a new frame since last
          // Get the last one in the list.
          var result = results.get(results.size() - 1);
          if (result != null && result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            // Get the best target (first one in the list)
            var target = result.getBestTarget();
            if (target != null) {
              targetYaw = target.getYaw();
              targetVisible = true;
              
              // Calculate distance to target using pitch angle
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
      // PhotonVision not connected or error reading camera
      // Continue with normal drive control
      targetVisible = false;
      targetYaw = 0.0;
      targetRange = 0.0;
      // Optionally log the error for debugging (commented out to avoid spam)
      // System.out.println("[ArcadeDrive] PhotonVision error: " + e.getMessage());
    }
    
    // Check button states
    boolean triangleButtonPressed = m_controller.getTriangleButton();
    boolean xButtonPressed = m_controller.getCrossButton();
    
    // Debug: Log when triangle button is pressed or released
    if (triangleButtonPressed && !m_previousTriangleButtonState) {
      System.out.println("[ArcadeDrive] Triangle button PRESSED - targetVisible: " + targetVisible);
    } else if (!triangleButtonPressed && m_previousTriangleButtonState) {
      System.out.println("[ArcadeDrive] Triangle button RELEASED");
    }
    m_previousTriangleButtonState = triangleButtonPressed;
    
    // Debug: Log when X button is pressed or released
    if (xButtonPressed && !m_previousXButtonState) {
      System.out.println("[ArcadeDrive] X button PRESSED - targetVisible: " + targetVisible + ", targetRange: " + targetRange);
    } else if (!xButtonPressed && m_previousXButtonState) {
      System.out.println("[ArcadeDrive] X button RELEASED");
    }
    m_previousXButtonState = xButtonPressed;
    
    // Auto-align when triangle button is pressed (only if X button is NOT pressed)
    // Use smooth rotation based on target angle
    if (triangleButtonPressed && !xButtonPressed) {
      // When first starting alignment AND target is visible, calculate the target angle
      if (!m_isAligning && targetVisible) {
        double currentAngle = m_drivetrain.getGyroAngleZ();
        // targetYaw is the angle from camera center to target
        // We want to rotate to align with the target, so target angle = current angle - targetYaw
        m_targetAngle = currentAngle - targetYaw;
        m_isAligning = true;
        m_previousAngleError = 0.0; // Reset derivative term
        System.out.println("[ArcadeDrive] Starting alignment: currentAngle=" + currentAngle + 
                          ", targetYaw=" + targetYaw + ", targetAngle=" + m_targetAngle);
      }
      
      // If we're actively aligning, continue rotating toward target angle
      if (m_isAligning && m_targetAngle != null) {
        // Calculate rotation based on error from target angle
        double currentAngle = m_drivetrain.getGyroAngleZ();
        double angleError = m_targetAngle - currentAngle;
        
        // Normalize angle error to [-180, 180] range
        while (angleError > 180) angleError -= 360;
        while (angleError < -180) angleError += 360;
        
        // If we're close enough, stop rotating
        if (Math.abs(angleError) <= Constants.Vision.kVisionAngleTolerance) {
          rotation = 0.0;
          m_previousAngleError = 0.0; // Reset derivative term
          System.out.println("[ArcadeDrive] Aligned! angleError=" + angleError);
        } else {
          // Calculate derivative term (rate of change of error)
          // This helps dampen oscillations by reducing rotation as we approach the target
          double errorDerivative = angleError - m_previousAngleError;
          
          // PD Controller: P term + D term
          double pTerm = angleError * Constants.Vision.kVisionTurnkP;
          double dTerm = errorDerivative * Constants.Vision.kVisionTurnkD;
          rotation = pTerm + dTerm;
          
          // Clamp to max rotation speed for smooth movement
          rotation = Math.max(-Constants.Vision.kVisionMaxRotation, 
                             Math.min(Constants.Vision.kVisionMaxRotation, rotation));
          
          // Apply minimum rotation threshold to overcome friction/deadband
          // Only apply if we're not already very close to the target
          if (Math.abs(angleError) > Constants.Vision.kVisionAngleTolerance * 2) {
            if (Math.abs(rotation) > 0 && Math.abs(rotation) < Constants.Vision.kVisionMinRotation) {
              rotation = Math.copySign(Constants.Vision.kVisionMinRotation, rotation);
            }
          }
          
          // Store current error for next iteration's derivative calculation
          m_previousAngleError = angleError;
        }
        
        SmartDashboard.putNumber("Vision/Target Angle", m_targetAngle);
        SmartDashboard.putNumber("Vision/Current Angle", currentAngle);
        SmartDashboard.putNumber("Vision/Angle Error", angleError);
      }
      // If button is pressed but we don't have a target yet, use normal rotation
    } else {
      // Button not pressed - reset alignment state
      if (m_isAligning) {
        m_targetAngle = null;
        m_isAligning = false;
        m_previousAngleError = 0.0;
        System.out.println("[ArcadeDrive] Stopped alignment");
      }
      // Use the normal rotation from the joystick (m_zaxisRotateSupplier)
    }
    
    // Use PathPlanner dynamic pathfinding when X button is pressed
    if (xButtonPressed && targetVisible) {
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
      
      // Use PathPlanner's AutoBuilder for dynamic pathfinding
      double currentTime = System.currentTimeMillis() / 1000.0;
      boolean needsReplan = false;
      
      // Check if we need to generate a new path
      if (m_lastTargetPose == null) {
        // No path currently planned
        needsReplan = true;
      } else {
        // Check if target moved significantly
        double targetMovement = m_lastTargetPose.getTranslation()
            .getDistance(targetPose.getTranslation());
        if (targetMovement > 0.15) { // 15cm threshold
          needsReplan = true;
        }
      }
      
      // Regenerate path periodically to adapt to changes (dynamic replanning)
      if (currentTime - m_lastPathGenerationTime > kPathRegenerationInterval) {
        needsReplan = true;
      }
      
      if (needsReplan) {
        try {
          // Create PathPlanner path from point A (current pose) to point B (target pose)
          PathConstraints constraints = new PathConstraints(
              Constants.PathPlanner.kMaxSpeedMetersPerSecond,
              Constants.PathPlanner.kMaxAccelerationMetersPerSecondSquared,
              Constants.PathPlanner.kMaxAngularSpeedRadiansPerSecond,
              Constants.PathPlanner.kMaxAngularAccelerationRadiansPerSecondSquared);
          
          // Create PathPlanner path from point A to point B
          // PathPlanner generates smooth Bézier curve paths between waypoints
          // We know where we are (currentPose) and where we want to go (targetPose)
          // PathPlanner will create the optimal smooth path between them
          
          // Create PathPlanner path from point A to point B
          // We know current pose (A) and target pose (B)
          // PathPlanner will generate a smooth path between them
          
          // Use PathPlanner to create path - this generates the optimal smooth trajectory
          // PathPlannerPath.fromPathPoints creates a path from waypoints
          // For a simple A-to-B path, we create waypoints at start and end
          m_currentPath = PathPlannerPath.fromPathPoints(
              java.util.List.of(
                  new PathPoint(
                      currentPose.getTranslation(), 
                      new RotationTarget(1.0, currentPose.getRotation())),
                  new PathPoint(
                      targetPose.getTranslation(), 
                      new RotationTarget(1.0, targetPose.getRotation()))
              ),
              constraints,
              new GoalEndState(0.0, targetPose.getRotation()));
          
          if (m_currentPath != null) {
            m_lastTargetPose = targetPose;
            m_lastPathGenerationTime = currentTime;
            
            System.out.println("[ArcadeDrive] PathPlanner path created from (" +
                String.format("%.2f", currentPose.getX()) + ", " + 
                String.format("%.2f", currentPose.getY()) + ") to (" +
                String.format("%.2f", targetPose.getX()) + ", " + 
                String.format("%.2f", targetPose.getY()) + ")");
          } else {
            System.out.println("[ArcadeDrive] PathPlanner path generation failed");
          }
        } catch (Exception e) {
          System.out.println("[ArcadeDrive] Error creating PathPlanner path: " + e.getMessage());
          e.printStackTrace();
          m_currentPath = null;
        }
      }
      
      // Follow the PathPlanner path smoothly
      // PathPlanner has calculated the optimal smooth path from A to B
      // We follow it by driving to the target with smooth velocity control
      if (m_currentPath != null && m_lastTargetPose != null) {
        // Calculate distance and angle to target
        Translation2d toTarget = m_lastTargetPose.getTranslation().minus(currentPose.getTranslation());
        double distanceToTarget = toTarget.getNorm();
        double angleToTarget = Math.atan2(toTarget.getY(), toTarget.getX());
        
        // Calculate desired velocity based on distance (smooth acceleration/deceleration)
        // Use PathPlanner's max speed, but scale down as we approach target
        double maxSpeed = Constants.PathPlanner.kMaxSpeedMetersPerSecond;
        double desiredSpeed = maxSpeed;
        
        // Slow down as we approach target (smooth deceleration)
        double slowDownDistance = 0.5; // Start slowing down 50cm from target
        if (distanceToTarget < slowDownDistance) {
          desiredSpeed = maxSpeed * (distanceToTarget / slowDownDistance);
        }
        
        // Calculate forward speed to reach desired velocity
        // Use the angle to target to determine forward direction
        Rotation2d robotHeading = currentPose.getRotation();
        double angleError = angleToTarget - robotHeading.getRadians();
        while (angleError > Math.PI) angleError -= 2 * Math.PI;
        while (angleError < -Math.PI) angleError += 2 * Math.PI;
        
        // Forward speed based on desired velocity and alignment
        double forwardSpeed = desiredSpeed * Math.cos(angleError);
        
        // Rotation speed to align with target
        double rotationError = m_lastTargetPose.getRotation().getRadians() - 
            currentPose.getRotation().getRadians();
        while (rotationError > Math.PI) rotationError -= 2 * Math.PI;
        while (rotationError < -Math.PI) rotationError += 2 * Math.PI;
        
        double rotationSpeed = m_pathRotationController.calculate(
            currentPose.getRotation().getRadians(),
            m_lastTargetPose.getRotation().getRadians());
        
        // Convert to normalized speeds [-1, 1]
        speed = (forwardSpeed / maxSpeed) * Constants.PathPlanner.kPowerMultiplier;
        rotation = rotationSpeed * Constants.PathPlanner.kPowerMultiplier;
        
        // Clamp to motor limits
        speed = Math.max(-1.0, Math.min(1.0, speed));
        rotation = Math.max(-1.0, Math.min(1.0, rotation));
        
        // Check if we've reached the target
        if (distanceToTarget < 0.1 && Math.abs(rotationError) < Units.degreesToRadians(5.0)) {
          // Reached target, stop
          speed = 0.0;
          rotation = 0.0;
          m_currentPath = null;
        }
      } else if (m_lastTargetPose != null) {
        // Path generation failed, fallback to direct approach
        double rangeError = Constants.Vision.kVisionDesiredRange - targetRange;
        double forwardSpeed = rangeError * Constants.PathPlanner.kTranslationkP;
        double rotationError = targetPose.getRotation().getRadians() - 
            currentPose.getRotation().getRadians();
        while (rotationError > Math.PI) rotationError -= 2 * Math.PI;
        while (rotationError < -Math.PI) rotationError += 2 * Math.PI;
        double rotationSpeed = m_pathRotationController.calculate(
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians());
        speed = forwardSpeed * Constants.PathPlanner.kPowerMultiplier;
        rotation = rotationSpeed * Constants.PathPlanner.kPowerMultiplier;
        speed = Math.max(-1.0, Math.min(1.0, speed));
        rotation = Math.max(-1.0, Math.min(1.0, rotation));
      }
      
      Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
      double distanceToTarget = toTarget.getNorm();
      SmartDashboard.putNumber("Vision/X-Button/PathPlanner/Distance to Target", distanceToTarget);
      SmartDashboard.putNumber("Vision/X-Button/PathPlanner/Target Range", targetRange);
      SmartDashboard.putBoolean("Vision/X-Button/PathPlanner/Pathfinding Active", 
          m_lastTargetPose != null);
    } else {
      // X button not pressed - cancel pathfinding and reset tracking
      m_currentPath = null;
      if (m_previousXButtonState) {
        m_previousAngleError = 0.0;
        m_previousRotationCommand = 0.0;
        m_previousSpeedCommand = 0.0;
        m_lastTargetPose = null;
      }
    }
    
    // Debug output
    SmartDashboard.putNumber("ArcadeDrive/Speed", speed);
    SmartDashboard.putNumber("ArcadeDrive/Rotation", rotation);
    SmartDashboard.putBoolean("ArcadeDrive/Is Running", true);
    SmartDashboard.putBoolean("ArcadeDrive/Triangle Button", triangleButtonPressed);
    SmartDashboard.putBoolean("ArcadeDrive/X Button", xButtonPressed);
    SmartDashboard.putBoolean("Vision/Target Visible", targetVisible);
    SmartDashboard.putNumber("Vision/Target Yaw", targetYaw);
    SmartDashboard.putNumber("Vision/Target Range", targetRange);
    SmartDashboard.putBoolean("Vision/Is Aligning", m_isAligning);
    
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("ArcadeDrive/Is Running", false);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
