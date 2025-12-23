// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForward extends Command {
  private final Drivetrain m_drive;
  private final double m_speed;
  private boolean m_firstExecution = true;

  /**
   * Creates a new DriveForward command. This command will drive the robot forward
   * at the specified speed until interrupted.
   *
   * @param speed The speed at which to drive forward
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveForward(double speed, Drivetrain drive) {
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("[DriveForward] Command initialized - speed: " + m_speed);
    m_firstExecution = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_firstExecution) {
      System.out.println("[DriveForward] First execution - driving forward at speed " + m_speed);
      m_firstExecution = false;
    }
    m_drive.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("[DriveForward] Command ended (interrupted: " + interrupted + ") - stopping motors");
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}

