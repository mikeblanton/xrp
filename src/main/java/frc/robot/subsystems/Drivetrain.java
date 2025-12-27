// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm
  private static final double kTrackWidthMeters = Units.inchesToMeters(6.0); // Approximate track width for XRP

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Pose estimator for tracking robot pose with vision support
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  // Kinematics for differential drive
  private final DifferentialDriveKinematics m_kinematics;

  // Field visualization
  private final Field2d m_field = new Field2d();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
    
    // Initialize kinematics first (needed for pose estimator)
    m_kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    
    // Initialize pose estimator at origin with standard deviations
    m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_kinematics,
        Rotation2d.fromDegrees(getGyroAngleZ()),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, 0.01), // State std devs (x, y, theta)
        VecBuilder.fill(0.5, 0.5, 0.5)); // Vision std devs (x, y, theta)
    
    // Set up field visualization
    SmartDashboard.putData("Field", m_field);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the XRP along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the XRP along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the XRP along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Get the current pose of the robot.
   * 
   * @return The current pose
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Reset the odometry to a specific pose.
   * 
   * @param pose The pose to reset to
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(getGyroAngleZ()),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        pose);
  }

  /**
   * Add a vision measurement to the pose estimator.
   * 
   * @param visionRobotPoseMeters The pose from vision
   * @param timestampSeconds The timestamp of the vision measurement
   * @param visionMeasurementStdDevs Standard deviations of the vision measurement
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, 
      edu.wpi.first.math.Matrix<N3, N1> visionMeasurementStdDevs) {
    m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /**
   * Get the Field2d object for visualization.
   * 
   * @return The Field2d object
   */
  public Field2d getField() {
    return m_field;
  }

  /**
   * Get the left encoder distance in meters.
   * 
   * @return Left distance in meters
   */
  public double getLeftDistanceMeters() {
    return Units.inchesToMeters(getLeftDistanceInch());
  }

  /**
   * Get the right encoder distance in meters.
   * 
   * @return Right distance in meters
   */
  public double getRightDistanceMeters() {
    return Units.inchesToMeters(getRightDistanceInch());
  }

  /**
   * Get the track width of the robot in meters.
   * 
   * @return Track width in meters
   */
  public double getTrackWidthMeters() {
    return kTrackWidthMeters;
  }

  /**
   * Get the current wheel speeds.
   * 
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_leftEncoder.getRate() * Units.inchesToMeters(1.0),
        m_rightEncoder.getRate() * Units.inchesToMeters(1.0));
  }

  /**
   * Drive the robot using tank drive (left and right speeds).
   * 
   * @param leftSpeed Left side speed (-1.0 to 1.0)
   * @param rightSpeed Right side speed (-1.0 to 1.0)
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Drive the robot using chassis speeds (for PathPlanner).
   * 
   * @param speeds Chassis speeds
   */
  public void driveChassisSpeeds(edu.wpi.first.math.kinematics.ChassisSpeeds speeds) {
    // Convert chassis speeds to wheel speeds
    var wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    
    // Convert wheel speeds to motor outputs
    // Normalize to max speed
    double maxWheelSpeed = Math.max(
        Math.abs(wheelSpeeds.leftMetersPerSecond),
        Math.abs(wheelSpeeds.rightMetersPerSecond));
    
    if (maxWheelSpeed > Constants.PathPlanner.kMaxSpeedMetersPerSecond) {
      wheelSpeeds.leftMetersPerSecond *= Constants.PathPlanner.kMaxSpeedMetersPerSecond / maxWheelSpeed;
      wheelSpeeds.rightMetersPerSecond *= Constants.PathPlanner.kMaxSpeedMetersPerSecond / maxWheelSpeed;
    }
    
    // Convert to normalized speeds [-1, 1]
    double leftOutput = wheelSpeeds.leftMetersPerSecond / Constants.PathPlanner.kMaxSpeedMetersPerSecond;
    double rightOutput = wheelSpeeds.rightMetersPerSecond / Constants.PathPlanner.kMaxSpeedMetersPerSecond;
    
    // Clamp to [-1, 1]
    leftOutput = Math.max(-1.0, Math.min(1.0, leftOutput));
    rightOutput = Math.max(-1.0, Math.min(1.0, rightOutput));
    
    m_diffDrive.tankDrive(leftOutput, rightOutput);
  }

  /**
   * Get the kinematics object.
   * 
   * @return The differential drive kinematics
   */
  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  @Override
  public void periodic() {
    // Update pose estimator with encoder and gyro data
    m_poseEstimator.update(
        Rotation2d.fromDegrees(getGyroAngleZ()),
        getLeftDistanceMeters(),
        getRightDistanceMeters());
    
    // Update field visualization
    m_field.setRobotPose(getPose());
    
    // Update SmartDashboard with debugging information
    SmartDashboard.putNumber("Drivetrain/Left Encoder Count", getLeftEncoderCount());
    SmartDashboard.putNumber("Drivetrain/Right Encoder Count", getRightEncoderCount());
    SmartDashboard.putNumber("Drivetrain/Left Distance (in)", getLeftDistanceInch());
    SmartDashboard.putNumber("Drivetrain/Right Distance (in)", getRightDistanceInch());
    SmartDashboard.putNumber("Drivetrain/Average Distance (in)", getAverageDistanceInch());
    SmartDashboard.putNumber("Drivetrain/Gyro Angle X", getGyroAngleX());
    SmartDashboard.putNumber("Drivetrain/Gyro Angle Y", getGyroAngleY());
    SmartDashboard.putNumber("Drivetrain/Gyro Angle Z", getGyroAngleZ());
    SmartDashboard.putNumber("Drivetrain/Accel X (G)", getAccelX());
    SmartDashboard.putNumber("Drivetrain/Accel Y (G)", getAccelY());
    SmartDashboard.putNumber("Drivetrain/Accel Z (G)", getAccelZ());
    SmartDashboard.putNumber("Drivetrain/Pose X (m)", getPose().getX());
    SmartDashboard.putNumber("Drivetrain/Pose Y (m)", getPose().getY());
    SmartDashboard.putNumber("Drivetrain/Pose Rotation (deg)", getPose().getRotation().getDegrees());
  }
}
