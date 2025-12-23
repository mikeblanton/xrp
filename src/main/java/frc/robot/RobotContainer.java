// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.DriveForward;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XRPOnBoardIO m_onboardIO = new XRPOnBoardIO();
  private final Arm m_arm = new Arm();

  // PS5 controller plugged into channel 0
  private final PS5Controller m_controller = new PS5Controller(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  // Debug logging state
  private int m_debugCounter = 0;
  private static final int DEBUG_LOG_INTERVAL = 25; // Log every 25 cycles (~500ms at 20ms/cycle)
  private boolean m_lastSquareButton = false;
  private boolean m_lastCrossButton = false;
  private boolean m_lastCircleButton = false;
  private boolean m_lastTriangleButton = false;
  private boolean m_lastL1Button = false;
  private boolean m_lastR1Button = false;
  private boolean m_lastPSButton = false;
  private boolean m_hasLoggedControllerInfo = false;
  private int m_controllerInfoRetryCount = 0;
  private static final int MAX_CONTROLLER_INFO_RETRIES = 50; // Retry for ~1 second

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.PS5Controller}), and then using {@link Trigger} with the controller's
   * button methods.
   */
  private void configureButtonBindings() {
    // Set default command to arcade drive using left joystick for both X and Y control
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Trigger userButton = new Trigger(m_onboardIO::getUserButtonPressed);
    userButton
        .onTrue(new PrintCommand("USER Button Pressed"))
        .onFalse(new PrintCommand("USER Button Released"));

    Trigger squareButton = new Trigger(m_controller::getSquareButton);
    squareButton
        .onTrue(new InstantCommand(() -> m_arm.setAngle(45.0), m_arm))
        .onFalse(new InstantCommand(() -> m_arm.setAngle(0.0), m_arm));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Apply deadband to controller input to ignore small values near zero.
   * 
   * @param value The input value
   * @param deadband The deadband threshold
   * @return The value with deadband applied, or 0 if below threshold
   */
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return value;
  }

  /**
   * Square the input value for smoother control at low speeds.
   * Preserves the sign of the input.
   * 
   * @param value The input value (-1.0 to 1.0)
   * @return The squared value with sign preserved
   */
  private double squareInput(double value) {
    return Math.copySign(value * value, value);
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   * Uses left joystick for both forward/backward (Y-axis) and left/right (X-axis) control.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, 
        () -> {
          // Left stick Y-axis for forward/backward speed
          double speed = -m_controller.getLeftY();
          speed = applyDeadband(speed, 0.1);
          speed = squareInput(speed);
          return speed;
        }, 
        () -> {
          // Left stick X-axis for left/right rotation
          double rotation = -m_controller.getLeftX();
          rotation = applyDeadband(rotation, 0.1);
          rotation = squareInput(rotation);
          return rotation;
        });
  }

  /**
   * Creates a command that drives the robot forward continuously.
   * 
   * @return a command that drives forward at a constant speed
   */
  public Command getDriveForwardCommand() {
    // Drive forward at speed 2 when X button is held
    // Note: Speed 2.0 is outside normal range (-1.0 to 1.0) but using as requested
    return new DriveForward(2.0, m_drivetrain);
  }

  /**
   * Update SmartDashboard with controller debugging information.
   * Call this periodically (e.g., from robotPeriodic).
   */
  public void updateControllerDebug() {
    // Debug logging temporarily disabled - only logging W key presses in trigger
    // Uncomment below to re-enable full controller debugging
    if (false) {
    // IMPORTANT: In Simulation UI, you must assign the controller in the "Joysticks" window:
    // 1. Open Simulation GUI
    // 2. Go to "Joysticks" tab/window  
    // 3. Assign your PS5 controller (from System Joysticks) to Port 0
    // 4. The controller should show as "Assigned" in the Joysticks window
    
    // Log detailed controller information - retry if no controllers found initially
    if (!m_hasLoggedControllerInfo || (m_controllerInfoRetryCount < MAX_CONTROLLER_INFO_RETRIES && 
        !DriverStation.isJoystickConnected(0))) {
      
      if (m_controllerInfoRetryCount == 0) {
        System.out.println("=== Controller Debug Information ===");
        System.out.println("Driver Station attached: " + DriverStation.isDSAttached());
        System.out.println("Is FMS attached: " + DriverStation.isFMSAttached());
        System.out.println("Robot enabled: " + DriverStation.isEnabled());
        System.out.println("Match time: " + DriverStation.getMatchTime());
        System.out.println("NOTE: Running in SIMULATION mode - joysticks come from Sim GUI, not Driver Station");
        System.out.println("Checking all joystick ports (0-5)...");
      }
      
      boolean foundAnyController = false;
      for (int port = 0; port < 6; port++) {
        boolean connected = DriverStation.isJoystickConnected(port);
        String name = DriverStation.getJoystickName(port);
        int axisCount = DriverStation.getStickAxisCount(port);
        int buttonCount = DriverStation.getStickButtonCount(port);
        int povCount = DriverStation.getStickPOVCount(port);
        boolean isXbox = DriverStation.getJoystickIsXbox(port);
        int type = DriverStation.getJoystickType(port);
        
        // Check if name suggests a controller is present (even if "connected" is false)
        boolean nameSuggestsController = !name.isEmpty() && 
            (name.contains("DualSense") || name.contains("Controller") || 
             name.contains("Xbox") || name.contains("Joystick") || 
             name.contains("Gamepad"));
        
        if (connected || nameSuggestsController || type != 255) {
          foundAnyController = true;
        }
        
        if (m_controllerInfoRetryCount == 0 || foundAnyController || nameSuggestsController) {
          System.out.printf("  Port %d: Connected=%s, Name='%s', Type=%d, Xbox=%s, Axes=%d, Buttons=%d, POVs=%d%n",
              port, connected, name, type, isXbox, axisCount, buttonCount, povCount);
          
          // If name suggests controller but connected is false, provide helpful info
          if (nameSuggestsController && !connected) {
            System.out.printf("    -> Controller name detected but not 'connected' - may need Driver Station enabled or port assignment%n");
          }
        }
        
        SmartDashboard.putBoolean("Controller/Port" + port + "/Connected", connected);
        SmartDashboard.putString("Controller/Port" + port + "/Name", name);
        SmartDashboard.putNumber("Controller/Port" + port + "/Type", type);
        SmartDashboard.putBoolean("Controller/Port" + port + "/IsXbox", isXbox);
        SmartDashboard.putNumber("Controller/Port" + port + "/AxisCount", axisCount);
        SmartDashboard.putNumber("Controller/Port" + port + "/ButtonCount", buttonCount);
      }
      
      if (foundAnyController || m_controllerInfoRetryCount >= MAX_CONTROLLER_INFO_RETRIES) {
        System.out.println("=====================================");
        if (!foundAnyController) {
          if (!DriverStation.isDSAttached()) {
            System.out.println("SIMULATION MODE: No Driver Station attached");
            System.out.println("  - Controllers are configured in the Simulation GUI (not Driver Station)");
            System.out.println("  - Open the Simulation GUI and go to the 'Joysticks' or 'Input' tab");
            System.out.println("  - Add/configure your PS5 controller in the simulation GUI");
            System.out.println("  - Make sure joystick forwarding is enabled in simulation settings");
            System.out.println("  - Controller should be assigned to port " + m_controller.getPort());
            System.out.println("  - Will attempt to read controller values directly...");
          } else {
            System.out.println("WARNING: No controllers detected by DriverStation!");
            System.out.println("  - Make sure Driver Station is running and connected");
            System.out.println("  - Check that controller is plugged in and recognized by OS");
            System.out.println("  - IMPORTANT: Driver Station must be ENABLED (not just running) for joysticks to work");
            System.out.println("  - Check Driver Station UI: Is the robot enabled? (Enable button should be pressed)");
          }
        } else {
          // Check if we found controllers by name but they're not "connected"
          for (int port = 0; port < 6; port++) {
            String name = DriverStation.getJoystickName(port);
            boolean connected = DriverStation.isJoystickConnected(port);
            if (!name.isEmpty() && (name.contains("DualSense") || name.contains("Controller")) && !connected) {
              System.out.printf("NOTE: Found '%s' on port %d but DriverStation reports not connected%n", name, port);
              System.out.println("  - This usually means Driver Station needs to be ENABLED");
              System.out.println("  - Or the controller needs to be assigned to port " + port + " in Driver Station UI");
              System.out.println("  - Will attempt to read controller values anyway...");
            }
          }
        }
        m_hasLoggedControllerInfo = true;
      } else {
        m_controllerInfoRetryCount++;
        if (m_controllerInfoRetryCount % 10 == 0) {
          System.out.printf("  (Retrying controller detection... attempt %d/%d)%n", 
              m_controllerInfoRetryCount, MAX_CONTROLLER_INFO_RETRIES);
        }
      }
    }
    
    // Controller connection status
    boolean isConnected = DriverStation.isJoystickConnected(0);
    SmartDashboard.putBoolean("Controller/Connected", isConnected);
    
    // Try to read controller values even if DriverStation says not connected
    // (In simulation, DriverStation won't report connected, but controller may still work)
    boolean canReadController = false;
    double testLeftY = 0.0;
    double testRightX = 0.0;
    boolean testXButton = false;
    try {
      testLeftY = m_controller.getLeftY();
      testRightX = m_controller.getRightX();
      testXButton = m_controller.getCrossButton();
      canReadController = true;
    } catch (Exception e) {
      canReadController = false;
      System.out.println("[Controller] ERROR reading controller values: " + e.getMessage());
    }
    
    SmartDashboard.putBoolean("Controller/CanReadValues", canReadController);
    SmartDashboard.putNumber("Controller/TestLeftY", testLeftY);
    SmartDashboard.putNumber("Controller/TestRightX", testRightX);
    SmartDashboard.putBoolean("Controller/XButtonRaw", testXButton);
    
    // Log X button state changes immediately (always check, not just periodically)
    // This runs every robot cycle, so we'll see button presses immediately
    if (testXButton != m_lastCrossButton) {
      System.out.println("=========================================");
      System.out.println("[Controller] X Button state changed: " + (testXButton ? "PRESSED" : "RELEASED"));
      System.out.println("[Controller]   - Can read controller: " + canReadController);
      System.out.println("[Controller]   - DriverStation connected: " + isConnected);
      System.out.println("[Controller]   - Controller port: " + m_controller.getPort());
      System.out.println("[Controller]   - DriverStation joystick name: " + DriverStation.getJoystickName(0));
      System.out.println("[Controller]   - DriverStation button count: " + DriverStation.getStickButtonCount(0));
      System.out.println("=========================================");
      m_lastCrossButton = testXButton;
    }
    
    // Also log periodically if button is held (every ~500ms)
    if (testXButton && m_debugCounter % 25 == 0) {
      System.out.println("[Controller] X Button is HELD (continuous check)");
    }
    
    // In simulation, always try to use controller even if DriverStation says not connected
    // Check if we're getting non-zero values (which means controller is actually working)
    boolean hasNonZeroInput = Math.abs(testLeftY) > 0.01 || Math.abs(testRightX) > 0.01;
    SmartDashboard.putBoolean("Controller/HasNonZeroInput", hasNonZeroInput);
    
    if (isConnected || canReadController) {
      // Joystick axes
      double leftX = m_controller.getLeftX();
      double leftY = m_controller.getLeftY();
      double rightX = m_controller.getRightX();
      double rightY = m_controller.getRightY();
      
      SmartDashboard.putNumber("Controller/Left Stick X", leftX);
      SmartDashboard.putNumber("Controller/Left Stick Y", leftY);
      SmartDashboard.putNumber("Controller/Right Stick X", rightX);
      SmartDashboard.putNumber("Controller/Right Stick Y", rightY);
      
      // Trigger values
      double leftTrigger = m_controller.getL2Axis();
      double rightTrigger = m_controller.getR2Axis();
      SmartDashboard.putNumber("Controller/Left Trigger", leftTrigger);
      SmartDashboard.putNumber("Controller/Right Trigger", rightTrigger);
      
      // Button states
      boolean squareButton = m_controller.getSquareButton();
      boolean crossButton = m_controller.getCrossButton();
      boolean circleButton = m_controller.getCircleButton();
      boolean triangleButton = m_controller.getTriangleButton();
      boolean l1Button = m_controller.getL1Button();
      boolean r1Button = m_controller.getR1Button();
      boolean psButton = m_controller.getPSButton();
      
      SmartDashboard.putBoolean("Controller/Square Button", squareButton);
      SmartDashboard.putBoolean("Controller/Cross Button", crossButton);
      SmartDashboard.putBoolean("Controller/Circle Button", circleButton);
      SmartDashboard.putBoolean("Controller/Triangle Button", triangleButton);
      SmartDashboard.putBoolean("Controller/L1 Button", l1Button);
      SmartDashboard.putBoolean("Controller/R1 Button", r1Button);
      SmartDashboard.putBoolean("Controller/PS Button", psButton);
      
      // Raw controller inputs
      double rawSpeed = -leftY;
      double rawRotation = -rightX;
      
      // Apply same processing as drive command
      double deadband = 0.1;
      double processedSpeed = Math.abs(rawSpeed) < deadband ? 0.0 : Math.copySign(rawSpeed * rawSpeed, rawSpeed);
      double processedRotation = Math.abs(rawRotation) < deadband ? 0.0 : Math.copySign(rawRotation * rawRotation, rawRotation);
      
      // Drive command inputs (what's actually being sent to the drivetrain)
      SmartDashboard.putNumber("Controller/Drive Speed (raw)", rawSpeed);
      SmartDashboard.putNumber("Controller/Drive Rotation (raw)", rawRotation);
      SmartDashboard.putNumber("Controller/Drive Speed (processed)", processedSpeed);
      SmartDashboard.putNumber("Controller/Drive Rotation (processed)", processedRotation);
      
      // Console logging - log button state changes immediately
      if (squareButton != m_lastSquareButton) {
        System.out.println("[Controller] Square Button: " + (squareButton ? "PRESSED" : "RELEASED"));
        m_lastSquareButton = squareButton;
      }
      if (crossButton != m_lastCrossButton) {
        System.out.println("[Controller] X Button (Cross): " + (crossButton ? "PRESSED" : "RELEASED"));
        m_lastCrossButton = crossButton;
      }
      if (circleButton != m_lastCircleButton) {
        System.out.println("[Controller] Circle Button: " + (circleButton ? "PRESSED" : "RELEASED"));
        m_lastCircleButton = circleButton;
      }
      if (triangleButton != m_lastTriangleButton) {
        System.out.println("[Controller] Triangle Button: " + (triangleButton ? "PRESSED" : "RELEASED"));
        m_lastTriangleButton = triangleButton;
      }
      if (l1Button != m_lastL1Button) {
        System.out.println("[Controller] L1 Button: " + (l1Button ? "PRESSED" : "RELEASED"));
        m_lastL1Button = l1Button;
      }
      if (r1Button != m_lastR1Button) {
        System.out.println("[Controller] R1 Button: " + (r1Button ? "PRESSED" : "RELEASED"));
        m_lastR1Button = r1Button;
      }
      if (psButton != m_lastPSButton) {
        System.out.println("[Controller] PS Button: " + (psButton ? "PRESSED" : "RELEASED"));
        m_lastPSButton = psButton;
      }
      
      // Console logging - log joystick values at reduced rate (every ~500ms)
      // Only log if there's actual input (non-zero values) to avoid console spam
      m_debugCounter++;
      if (m_debugCounter >= DEBUG_LOG_INTERVAL) {
        m_debugCounter = 0;
        // Check if there's any meaningful input (above deadband threshold)
        boolean hasInput = Math.abs(rawSpeed) > 0.1 || Math.abs(rawRotation) > 0.1 || 
                          Math.abs(leftX) > 0.1 || Math.abs(leftY) > 0.1 || 
                          Math.abs(rightX) > 0.1 || Math.abs(rightY) > 0.1;
        if (hasInput) {
          System.out.printf("[Controller] Left Stick: (%.3f, %.3f) | Right Stick: (%.3f, %.3f) | " +
              "Speed: %.3f (raw) -> %.3f (processed) | Rotation: %.3f (raw) -> %.3f (processed)%n",
              leftX, leftY, rightX, rightY,
              rawSpeed, processedSpeed, rawRotation, processedRotation);
        }
      }
    } else {
      // Log connection status change with more details
      int controllerPort = m_controller.getPort();
      if (m_debugCounter == 0) {
        String joystickName = DriverStation.getJoystickName(controllerPort);
        System.out.printf("[Controller] WARNING: DriverStation reports controller NOT connected on port %d%n", controllerPort);
        if (!DriverStation.isDSAttached()) {
          System.out.printf("  SIMULATION MODE: Controller must be configured in Simulation GUI%n");
          System.out.printf("  - Open Simulation GUI -> Joysticks/Input tab%n");
          System.out.printf("  - Add controller and assign to port %d%n", controllerPort);
        } else {
          System.out.printf("  But joystick name reported as: '%s'%n", joystickName);
          System.out.printf("  Axis count: %d, Button count: %d%n", 
              DriverStation.getStickAxisCount(controllerPort), DriverStation.getStickButtonCount(controllerPort));
        }
        System.out.printf("  Attempting to read controller values anyway...%n");
      }
      m_debugCounter++;
      if (m_debugCounter >= DEBUG_LOG_INTERVAL) {
        m_debugCounter = 0;
        // Try to read a value to see if controller actually works
        try {
          double testLeftYValue = m_controller.getLeftY();
          double testRightXValue = m_controller.getRightX();
          boolean hasInput = Math.abs(testLeftYValue) > 0.01 || Math.abs(testRightXValue) > 0.01;
          if (hasInput) {
            System.out.printf("[Controller] SUCCESS: Controller is working! LeftY=%.3f, RightX=%.3f%n", 
                testLeftYValue, testRightXValue);
          } else {
            System.out.printf("[Controller] NOTE: Can read controller but values are zero (LeftY=%.3f, RightX=%.3f)%n", 
                testLeftYValue, testRightXValue);
            if (!DriverStation.isDSAttached()) {
              System.out.printf("  -> In simulation: Make sure controller is configured in Simulation GUI%n");
            }
          }
        } catch (Exception e) {
          System.out.printf("[Controller] ERROR: Cannot read controller values: %s%n", e.getMessage());
        }
      }
    }
    }
  }
}
