// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void robotInit() {
    // Suppress joystick unplugged warnings during initialization
    // This is a known issue with PS5 controllers where Driver Station
    // may check button availability before the controller is fully ready
    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Option 1: NetworkTables server runs on Mac (automatically started by WPILib simulation)
    // The server is already running - logs show "NT: Listening on NT3 port 1735, NT4 port 5810"
    // PhotonVision (running on Raspberry Pi) needs to connect to this server on the Mac
    String macIpAddress = null;
    try {
      var interfaces = java.net.NetworkInterface.getNetworkInterfaces();
      while (interfaces.hasMoreElements() && macIpAddress == null) {
        var ni = interfaces.nextElement();
        var addresses = ni.getInetAddresses();
        while (addresses.hasMoreElements() && macIpAddress == null) {
          var addr = addresses.nextElement();
          if (!addr.isLoopbackAddress() && addr instanceof java.net.Inet4Address) {
            macIpAddress = addr.getHostAddress();
          }
        }
      }
    } catch (Exception e) {
      // Could not detect IP, will use fallback message
    }
    
    System.out.println("========================================");
    System.out.println("[Robot] NetworkTables Configuration:");
    System.out.println("[Robot] Team Number: 6619");
    System.out.println("[Robot] Server mode: Enabled (automatically started)");
    if (macIpAddress != null) {
      System.out.println("[Robot] Mac IP Address: " + macIpAddress);
    } else {
      System.out.println("[Robot] Mac IP Address: (run 'ifconfig' to find it)");
    }
    System.out.println("[Robot] NT4 port: 5810");
    System.out.println("[Robot] NT3 port: 1735");
    System.out.println("[Robot]");
    System.out.println("[Robot] PhotonVision Configuration (on Raspberry Pi):");
    System.out.println("[Robot]   1. Open PhotonVision web UI (on the Raspberry Pi)");
    System.out.println("[Robot]   2. Go to Settings -> Network");
    if (macIpAddress != null) {
      System.out.println("[Robot]   3. Set 'NetworkTables Server Address' to: " + macIpAddress);
    } else {
      System.out.println("[Robot]   3. Set 'NetworkTables Server Address' to your Mac's IP address");
    }
    System.out.println("[Robot]      (NOT localhost - use the Mac's IP address)");
    System.out.println("[Robot]   4. Save and restart PhotonVision if needed");
    System.out.println("[Robot]");
    System.out.println("[Robot] Note: TimeSyncServer warnings in simulation are common");
    System.out.println("[Robot]       and usually harmless. PhotonVision will still work,");
    System.out.println("[Robot]       though timestamps may be slightly less accurate.");
    System.out.println("========================================");
    
    // NetworkTables TimeSyncServer should be automatically enabled in server mode
    // In simulation, there may be warnings but functionality is not affected
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    // Periodically check NetworkTables connection status
    checkNetworkTablesStatus();
  }
  
  private int m_networkTablesCheckCounter = 0;
  
  /**
   * Checks NetworkTables server status and PhotonVision visibility.
   * Runs every 50 cycles (about once per second) to avoid spam.
   * Note: isConnected() returns false for servers (they accept connections, not connect to others)
   */
  private void checkNetworkTablesStatus() {
    m_networkTablesCheckCounter++;
    if (m_networkTablesCheckCounter >= 50) {
      m_networkTablesCheckCounter = 0;
      
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      // For servers, isConnected() always returns false (servers don't "connect")
      // Instead, check if PhotonVision has connected by looking for its tables
      
      // Check if PhotonVision table exists (indicates PhotonVision has connected)
      var photonVisionTable = inst.getTable("photonvision");
      boolean photonVisionTableExists = photonVisionTable != null && 
                                        photonVisionTable.getKeys().size() > 0;
      
      // Check for the specific camera table
      var cameraTable = inst.getTable("photonvision/" + Constants.Vision.kCameraName);
      boolean cameraTableExists = cameraTable != null && cameraTable.getKeys().size() > 0;
      
      SmartDashboard.putBoolean("NetworkTables/Server Running", true);
      SmartDashboard.putBoolean("NetworkTables/PhotonVision Connected", photonVisionTableExists);
      SmartDashboard.putBoolean("NetworkTables/Camera Table Exists", cameraTableExists);
      
      if (!photonVisionTableExists) {
        System.out.println("[Robot] INFO: Waiting for PhotonVision to connect to NetworkTables server");
        System.out.println("[Robot] Make sure PhotonVision is running and configured to connect to localhost");
      }
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    System.out.println("[Robot] teleopInit() called");
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      System.out.println("[Robot] Cancelling autonomous command");
      m_autonomousCommand.cancel();
    }
    
    // The default command (arcade drive) will automatically start running
    // since it's set as the default command for the drivetrain subsystem
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
