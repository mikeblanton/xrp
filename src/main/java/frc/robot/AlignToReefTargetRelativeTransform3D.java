package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * Robot approach target and drive to requested pose based on the relative position to the target.
 * That is, this command drives to a Transform3d coordinate location near the target as opposed to
 * the {@link AlignToReefFieldRelativePose3D} which drives to a field location coordinate.
 * 
 * <p>This example uses chassis speed.
 * 
 * <p>Pick what the drivetrain interface actually needs.
 * 
 * <p>This is an example for any AprilTag in view. Validation of the tag being used must be done
 * and then related to where the target is in relation to the tag. This scheme worked for several
 * targets in ReefScape 2025 because several of the scoring positions where identically relative
 * to their designated tags. Validation had to be done to assure it was one of those in the set of
 * identically relative targets. That tag selection and validation is simplistic in this example.
 * 
 * <p>Thus, in use, this command must have a list of (tags, Transform3d) that are the coordinates of
 * the robot's scoring positions relative to each of the corresponding tags. This example has one
 * tag and two scoring positions - nothing fancy for validation.
 * 
 * <p>Related references:
 * Started with a view of BetaBot2025-develop[ElectroBunny].zip
 *    3 PIDs to align robot to reef tag 10 left or right branches
 *    https://www.chiefdelphi.com/t/from-zero-to-auto-align-to-reef-tutorial-how-to-simply-auto-align-to-the-reef/494478
 * 
 * <p>Other references for 3-D poses at LimelightVision.io and photonvision.org
 */
public class AlignToReefTargetRelativeTransform3D extends Command {
  static
  {
    System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
  }
  
  private int tagIDDesired;

  // PID constants - increased for more aggressive control
  private static final double X_REEF_ALIGNMENT_KP = 2.0; // Increased for better response
  private static final double X_REEF_ALIGNMENT_KI = 0.2; // Increased integral to eliminate steady-state error
  private static final double Y_REEF_ALIGNMENT_KP = 1.5;
  private static final double Y_REEF_ALIGNMENT_KI = 0.15; // Integral term to eliminate steady-state error
  private static final double ROT_REEF_ALIGNMENT_KP = 2.0; // Increased for rotation
  private static final double ROT_REEF_ALIGNMENT_KI = 0.2; // Increased integral to eliminate steady-state error

  // Tolerances - robot must be close to target
  private static final double X_TOLERANCE_REEF_ALIGNMENT = 0.05; // meters (5cm)
  private static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.05; // meters (5cm)
  private static final double ROT_TOLERANCE_REEF_ALIGNMENT = Units.degreesToRadians(3.); // 3 degrees
  
  // Minimum output values to overcome friction and deadband - increased significantly
  private static final double MIN_OUTPUT_THRESHOLD = 0.2; // Increased to ensure movement even with small errors

  private PIDController xController;
  private PIDController yController;
  private PIDController rotController;

  private static final double DONT_SEE_TAG_WAIT_TIME = 10.; //FIXME that's a long time to drive without knowing where to go; maybe stop motors after .25 sec but keep the command going for 1.?
  // odometry should take over if there is no vision after a short time like 1 iteration? no vision then use odometry?
  private static final double HOLD_POSE_VALIDATION_TIME = 0.3;

  private Timer dontSeeTagTimer;
  private Timer holdPoseValidationTimer;
  private boolean wasAtSetpoint; // Track previous setpoint state to reset timer on transition
  private final Transform3d targetBranch; // robot scoring position offset from AprilTag
  private Transform3d target;
  private VisionContainer visionContainer;
  private Drivetrain drivetrain;
  private boolean bail;
  private double xSpeed;
  private double ySpeed;
  private double rotValue;

  public AlignToReefTargetRelativeTransform3D(boolean isRightScore, VisionContainer visionContainer, Drivetrain drivetrain) {

    this.visionContainer = visionContainer;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    //FIXME To score, need a list of the robot's scoring positions relative to the tags
    
    // Simple example with one tag that has two similar scoring positions so take advantage of that symmetry
    tagIDDesired = 1; // example is short list that isn't even a list

    // Target: 0.75 meters in front of tag, with tag centered in camera (Y=0), facing the tag
    // X: 0.75m forward (tag is 0.75m in front of camera)
    // Y: 0.0m centered (tag centered laterally in camera view)
    // Rotation: π radians (180°) - robot facing the tag
    targetBranch = new Transform3d(0.75, 0.0, 0., new Rotation3d(0., 0., Math.PI));
    // Note: isRightScore parameter is ignored since we want centered positioning

    target = targetBranch;

    xController = new PIDController(X_REEF_ALIGNMENT_KP, X_REEF_ALIGNMENT_KI, 0); // +forward/-back translation
    xController.setSetpoint(target.getX());
    xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);
    xController.setIntegratorRange(-0.8, 0.8); // Increased integrator range to allow more integral accumulation

    yController = new PIDController(Y_REEF_ALIGNMENT_KP, Y_REEF_ALIGNMENT_KI, 0); // +left/-right translation
    yController.setSetpoint(target.getY());
    yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);
    yController.setIntegratorRange(-0.5, 0.5); // Limit integral windup

    rotController = new PIDController(ROT_REEF_ALIGNMENT_KP, ROT_REEF_ALIGNMENT_KI, 0); // +CCW/-CW rotation
    rotController.setSetpoint(target.getRotation().getZ());
    rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    rotController.setIntegratorRange(-0.8, 0.8); // Increased integrator range to allow more integral accumulation
  }

  @Override
  public void initialize() {

    holdPoseValidationTimer = new Timer();
    holdPoseValidationTimer.start();
    dontSeeTagTimer = new Timer();
    dontSeeTagTimer.start();
    bail = false;
    wasAtSetpoint = false;
    xController.reset();
    yController.reset();
    rotController.reset();
  }

  @Override
  public void execute() {

    RobotPose pose;

    if (visionContainer.getRobotPose().isPresent())
    { // see a tag so reset countdown to failure timer and process this iteration
      pose = visionContainer.getRobotPose().get();
      dontSeeTagTimer.reset();      
    }
    else
    { // no tag seen so let the countdown to failure timer run and quit this iteration
      return;
    }

    if (pose.AprilTagId != tagIDDesired) // make sure still looking at the correct tag
    {
      System.out.println("Oops! Looking at wrong tag " + pose.AprilTagId);
      bail = true;
      return;
    }    

    if (pose.pose3D.equals(Pose3d.kZero)) // make sure there is 3-D vision; this is a cheat that works most of the time
    {
      System.out.println("Oops! 3-D processing mode not activated in selected vision system");
      bail = true;
      return;
    }

    // Because the robot pose is transform3d from robot to tag, the PID controller must be in reverse
    // Too big robot position means drive forward at +speed to reduce the -delta which is Setpoint-Measurement

    double xError = pose.cameraToTarget.getX() - target.getX();
    double yError = pose.cameraToTarget.getY() - target.getY();
    double rotError = pose.cameraToTarget.getRotation().getZ() - target.getRotation().getZ();
    
    System.out.println(String.format("Target: X=%.3f Y=%.3f Rot=%.3f | Current: X=%.3f Y=%.3f Rot=%.3f | Error: X=%.3f Y=%.3f Rot=%.3f",
        target.getX(), target.getY(), target.getRotation().getZ(),
        pose.cameraToTarget.getX(), pose.cameraToTarget.getY(), pose.cameraToTarget.getRotation().getZ(),
        xError, yError, rotError));

    xSpeed = -xController.calculate(pose.cameraToTarget.getX());
    ySpeed = -yController.calculate(pose.cameraToTarget.getY());
    double baseRotValue = -rotController.calculate(pose.cameraToTarget.getRotation().getZ());
    
    // For differential drive, Y (lateral) error must be converted to rotation
    // Y+ means tag is left of camera (from RobotPose comments)
    // To center: if tag is left (Y+), rotate left to point at it
    // Rotation: positive is CCW (left), negative is CW (right)
    // So: if Y error is positive (tag left), we want positive rotation (rotate left)
    //     if Y error is negative (tag right), we want negative rotation (rotate right)
    double absYError = Math.abs(yError);
    
    // Convert Y error directly to rotation correction - simpler and more direct
    // Scale Y error aggressively to ensure centering
    double yErrorScale = 1.5; // Base scale for Y to rotation conversion
    if (absYError > 0.15) {
      yErrorScale = 2.5; // Very aggressive when off-center
    }
    if (absYError > 0.25) {
      yErrorScale = 3.5; // Extremely aggressive when way off-center
    }
    
    double yErrorCorrection = yError * yErrorScale;
    
    // Combine rotation PID with Y error correction
    // Prioritize Y correction when Y error is significant
    if (absYError > 0.1) {
      // When Y error is significant, mostly use Y correction for centering
      rotValue = yErrorCorrection * 0.9 + baseRotValue * 0.1;
    } else {
      // When Y error is small, combine both
      rotValue = baseRotValue + yErrorCorrection * 0.5;
    }
    
    // Only reduce forward speed when Y error is very large - allow more forward progress
    if (absYError > 0.25) {
      xSpeed *= 0.7; // Moderate reduction only when very off-center
    } else if (absYError > 0.2) {
      xSpeed *= 0.85; // Less reduction
    }
    
    // Debug: log PID outputs before applying minimum threshold
    System.out.println(String.format("PID outputs (before threshold): X=%.3f Y=%.3f Rot=%.3f (base=%.3f, Y-corr=%.3f, Y-scale=%.2f) | At setpoint: X=%b Y=%b Rot=%b",
        xSpeed, ySpeed, rotValue, baseRotValue, yErrorCorrection, yErrorScale,
        xController.atSetpoint(), yController.atSetpoint(), rotController.atSetpoint()));
    
    // Apply minimum threshold to overcome friction/deadband when error is significant
    // Always apply minimum threshold if error exceeds tolerance, regardless of PID output size
    // For X: apply threshold if error exceeds tolerance
    if (Math.abs(xError) > X_TOLERANCE_REEF_ALIGNMENT) {
      if (Math.abs(xSpeed) < MIN_OUTPUT_THRESHOLD) {
        xSpeed = Math.copySign(MIN_OUTPUT_THRESHOLD, xSpeed);
      }
    }
    
    // Y speed is converted to rotation, so don't apply threshold here
    // For rotation: apply threshold if rotation error exceeds tolerance OR if Y error is significant
    // (because rotation is used to center the tag)
    double absRotError = Math.abs(rotError);
    if (absRotError > Math.PI) {
      absRotError = 2 * Math.PI - absRotError;
    }
    // Apply minimum threshold if rotation error exceeds tolerance OR if we need to center (Y error > 0.05)
    if ((absRotError > ROT_TOLERANCE_REEF_ALIGNMENT || absYError > 0.05) && 
        Math.abs(rotValue) < MIN_OUTPUT_THRESHOLD) {
      rotValue = Math.copySign(MIN_OUTPUT_THRESHOLD, rotValue);
    }
    
    // Debug: log final outputs after threshold
    System.out.println(String.format("Final outputs: X=%.3f Y=%.3f Rot=%.3f",
        xSpeed, ySpeed, rotValue));
    // These pose calculate() use the camera which generally is slow to respond
    // (limelight claims to the contrary not withstanding).
    // Generally it's superior to use the robot pose as the best estimate using
    // odometry, gyro, and vision.
    // Also, odometry would be available if vision wasn't available because of slow
    // updates or no longer in view.
   }

  @Override
  public void end(boolean interrupted) {
    drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));

    if (interrupted)
    {
        System.out.println(" ended by interrupted ");
    }
  }

  @Override
  public boolean isFinished() {
    // Check if we're at the setpoint
    boolean atSetpoint = rotController.atSetpoint() &&
                         yController.atSetpoint() &&
                         xController.atSetpoint();

    if (atSetpoint) {
        // at goal pose so stop and see if it settles
        drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        
        // If we just reached setpoint (transition), reset the timer to start counting
        if (!wasAtSetpoint) {
          holdPoseValidationTimer.reset();
          wasAtSetpoint = true;
        }
    }
    else {
      // Not at setpoint, so drive and reset the timer
      drivetrain.driveRobotRelative(new ChassisSpeeds(xSpeed, ySpeed, rotValue));
      holdPoseValidationTimer.reset();
      wasAtSetpoint = false;
    }

    var dontSeeTag = dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME);
    // Only check holdPose timer when we're actually at setpoint
    var holdPose = atSetpoint && holdPoseValidationTimer.hasElapsed(HOLD_POSE_VALIDATION_TIME);

    if (bail)
    {
      System.out.print(" ended by bail ");
    }

    if (dontSeeTag)
    {
      System.out.print(" ended by don't see tag ");
    }

    if (holdPose)
    {
      System.out.print(" ended by correct pose held ");
    }

    return bail || dontSeeTag || holdPose;
  }
}
