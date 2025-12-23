// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final XRPServo m_armServo;
  private double m_currentAngle = 0.0;

  /** Creates a new Arm. */
  public Arm() {
    // Device number 4 maps to the physical Servo 1 port on the XRP
    m_armServo = new XRPServo(4);
  }

  @Override
  public void periodic() {
    // Update SmartDashboard with debugging information
    SmartDashboard.putNumber("Arm/Current Angle (deg)", m_currentAngle);
  }

  /**
   * Set the current angle of the arm (0 - 180 degrees).
   *
   * @param angleDeg Desired arm angle in degrees
   */
  public void setAngle(double angleDeg) {
    m_currentAngle = angleDeg;
    m_armServo.setAngle(angleDeg);
  }
  
  /**
   * Get the current angle of the arm.
   *
   * @return Current arm angle in degrees
   */
  public double getAngle() {
    return m_currentAngle;
  }
}
