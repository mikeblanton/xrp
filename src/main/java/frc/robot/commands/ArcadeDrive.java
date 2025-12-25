// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

public class ArcadeDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;
  private final PhotonCamera m_camera;
  private final PS5Controller m_controller;
  
  // Track previous triangle button state to detect presses/releases
  private boolean m_previousTriangleButtonState = false;
  
  // Track target angle for smooth rotation
  private Double m_targetAngle = null; // null means not actively aligning
  private boolean m_isAligning = false;
  private double m_previousAngleError = 0.0; // For derivative term calculation

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
            }
          }
        }
      }
    } catch (Exception e) {
      // PhotonVision not connected or error reading camera
      // Continue with normal drive control
      targetVisible = false;
      targetYaw = 0.0;
      // Optionally log the error for debugging (commented out to avoid spam)
      // System.out.println("[ArcadeDrive] PhotonVision error: " + e.getMessage());
    }
    
    // Check triangle button state for debugging
    boolean triangleButtonPressed = m_controller.getTriangleButton();
    
    // Debug: Log when triangle button is pressed or released
    if (triangleButtonPressed && !m_previousTriangleButtonState) {
      System.out.println("[ArcadeDrive] Triangle button PRESSED - targetVisible: " + targetVisible);
    } else if (!triangleButtonPressed && m_previousTriangleButtonState) {
      System.out.println("[ArcadeDrive] Triangle button RELEASED");
    }
    m_previousTriangleButtonState = triangleButtonPressed;
    
    // Auto-align when triangle button is pressed
    // Use smooth rotation based on target angle
    if (triangleButtonPressed) {
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
    
    // Debug output
    SmartDashboard.putNumber("ArcadeDrive/Speed", speed);
    SmartDashboard.putNumber("ArcadeDrive/Rotation", rotation);
    SmartDashboard.putBoolean("ArcadeDrive/Is Running", true);
    SmartDashboard.putBoolean("ArcadeDrive/Triangle Button", triangleButtonPressed);
    SmartDashboard.putBoolean("Vision/Target Visible", targetVisible);
    SmartDashboard.putNumber("Vision/Target Yaw", targetYaw);
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
