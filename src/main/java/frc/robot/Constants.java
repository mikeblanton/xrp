// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Vision {
    public static final String kCameraName = "Arducam_OV9281_USB_Camera";
    public static final double kVisionTurnkP = 0.15; // Proportional gain for rotation (increased for stronger response while maintaining smoothness)
    public static final double kVisionTurnkD = 0.08; // Derivative gain to dampen oscillations
    public static final double kVisionMaxRotation = 1.0; // Maximum rotation speed (full power)
    public static final double kVisionMinRotation = 0.2; // Minimum rotation to overcome friction/deadband
    public static final double kVisionAngleTolerance = 2.0; // Degrees - stop rotating when within this tolerance
    public static final double kVisionRotationRateLimit = 0.2; // Maximum change in rotation per cycle (for smoothing)
    
    // Range control constants
    public static final double kVisionRangekP = 0.9; // Proportional gain for forward/backward range control (increased for stronger response)
    public static final double kVisionMaxSpeed = 1.0; // Maximum forward/backward speed (full power)
    public static final double kVisionRangeTolerance = 0.1; // Meters - stop moving when within this tolerance
    public static final double kVisionDesiredRange = 1.0; // Meters - desired distance from target
    public static final double kVisionSpeedRateLimit = 0.18; // Maximum change in speed per cycle (for smoothing)
    
    // Camera and target physical constants (for distance calculation)
    public static final double kCameraMountHeightMeters = 0.5; // Height of camera above ground (meters) - adjust to your robot
    public static final double kTargetHeightMeters = 0.54; // Height of AprilTag center above ground (meters) - adjust to your target
    public static final double kCameraPitchDegrees = -30.0; // Camera pitch angle (degrees, negative if angled down) - adjust to your setup
  }

  public static class PathPlanner {
    // Path following controller gains (aggressively increased for maximum power)
    public static final double kTranslationkP = 15.0; // Much higher for strong response
    public static final double kTranslationkI = 0.0;
    public static final double kTranslationkD = 0.0;
    
    public static final double kRotationkP = 15.0; // Much higher for strong response
    public static final double kRotationkI = 0.0;
    public static final double kRotationkD = 0.0;
    
    // Power multiplier - directly scales PID output to motor power
    // This bypasses normalization to give full motor power
    public static final double kPowerMultiplier = 2.0; // Multiply PID output by this for more power
    
    // Max speeds and accelerations (increased for more power)
    public static final double kMaxSpeedMetersPerSecond = 5.0; // Max speed set to 5
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.0; // Increased from 0.5
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2; // Increased from Math.PI
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI * 2; // Increased from Math.PI
    
    // Replanning configuration
    public static final double kReplanningMaxDeviationMeters = 0.5; // Max deviation before replanning
    public static final double kReplanningMaxAngleErrorRadians = Math.toRadians(30.0); // Max angle error before replanning
  }
}
