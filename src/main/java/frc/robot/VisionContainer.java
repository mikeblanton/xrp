package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

/**
 * This class is a wrapper for the three vision classes {@link ControllerVision}, {@link PhotonVision}, and {@link LimelightVision}
 * <p>ControllerVision (roboRIO) returns team specified pose of lowest tag ID (this needs work; it'll be wrong a lot).
 * <p>PhotonVision returns its best pose (but not the very best pose estimator that includes odometry and gyro that teams should add).
 * <p>LimelightVision returns team specified filtered selection from MegaTag or MegaTag2.
 * (This is a reasonable default but could use more tuning. See {@link LimelightVision#update()})
 * 
 * <p>After selecting and configuring the desired vision system, use the robot pose as in example commands
 * {@link AlignToReefFieldRelativePose3D}, {@link AlignToReefTargetRelativeTransform3D} and {@link AlignToReefTagRelativeArcade2D}
 * 
 * <p>Getters are available for each of the three vision systems. This allows use of other low level methods that may be available
 * within the systems' classes. That also helps reveal the following getters which are duplicative with the RobotPose class data so
 * there is no reason to use them.
 * <p>Each vision system extends CameraBase which provides for
 * <pre><code>
 *public abstract void update();
 *public abstract boolean isFresh();
 *public abstract int getTagID();
 *public abstract Pose3d getPose3d();
 *public abstract Pose2d getPose2d();
 *public abstract double getTX();
 *public abstract double getTY();
 *public abstract Transform3d getCameraToTarget();
 * </code></pre>
 * <p>Usage of these methods requires that after "update()" the "isFresh()" is checked before using any of the getters.
 * <p>Since all of these data are available in the RobotPose there isn't a reason to use these methods in this example project. Other projects may use them.
 */
public class VisionContainer {
  static {
    System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName()); // optional show class has started

    System.loadLibrary(org.opencv.core.Core.NATIVE_LIBRARY_NAME); // OpenCV required for all Vision Processes

    try { // likely needed for 3-D pose usage
      Class.forName("frc.robot.AprilTagsLocations");
    } catch (ClassNotFoundException e) {
      e.printStackTrace();
    }
  }

  public enum VisionSelector{useControllerVision, usePhotonVision, useLimelightVision}
  private VisionSelector visionSelector;

  private ControllerVision controllerVision = null; // only for Controller (roboRIO)
  private PhotonVision photonVision = null; // only for PV
  private LimelightVision limelightVision = null; // only for LL
  private AtomicReference<Optional<RobotPose>> robotPose = new AtomicReference<Optional<RobotPose>>(Optional.empty());
  private Transform3d robotToCamera;
  private boolean vision = false; // initially not activated

  public VisionContainer(VisionSelector visionSelector)
  {
    this.visionSelector = visionSelector;

    switch (visionSelector)
    {
    case useControllerVision:
      /*
      Select the camera model and resolution from the list of cameras
      that have calibration data.

      Select if the Pose3D calculation is to be performed.
      
      Enter the 3-D translation (location) of your camera wrt the bottom, center of the robot.
 
      Caution: roboRIO v1 has constrained memory and cpu.

      For v1 use camera resolution about 320x240 maybe a little higher.
      Experiment with your team usage.

      "usePose3D" determines if the time consuming calculation for 3-D pose is performed.
      If this setting is "false", then only the pitch and yaw of the robot wrt the AprilTag is
      available and can they can be used for what LimeLight describes as "servoing" mode. Note
      that the Pose3d is still "available" but it's value is zero (origin; not useful).
      roboRIO v1 can do Pose3D but little time is left for other processes.
      v1 memory limits AprilTag bitsCorrected to 1 error bit (up to 2 on roboRIO v2)

      Select the camera device id

      Enter the camera location wrt the robot. Example camera mounted facing
      forward (zero radians yaw), half meter forward of center, quarter meter up from center,
      zero meters left of center, pitch pointing up 25 degrees. (camera pointing up is "-""; down is "+"")

      If in Simulation mode on a Windows PC, usually this is correct for camera id:
        *If the external USB camera is plugged in at (before) boot-up time,
        then it is cameraDeviceId = 0 after booting.
        *If the PC has an internal camera (common with laptops) and the
        external camera is plugged in after boot-up, then the cameraDeviceId = 1.
      */

      var cameraDeviceId = 0; // 1; if only one camera, it doesn't matter which physical USB port is used - both are 0
      if (ControllerVision.isAvailable(cameraDeviceId))
      {
        boolean usePose3D = true;
        // Camera is centered on front of robot, 3.6875" forward from center, 2" above center, mounted flat
        var robotToCameraCV = new Transform3d(new Translation3d(Units.inchesToMeters(3.1875), 0.0, Units.inchesToMeters(2.0)),
                  new Rotation3d(0, 0, 0));
        robotToCamera = robotToCameraCV;
        controllerVision = new ControllerVision(cameraDeviceId, ControllerVision.CameraOption.ArduCam320x240, usePose3D, robotToCameraCV);

        // vision runs as a "background" thread to minimize interference with the other
        // robot actions. This may increase latency. Priority could be experimented with.
        Thread acquireControllerCamera = new Thread(controllerVision);
        acquireControllerCamera.setName("roboRIOCameraPose");
        acquireControllerCamera.setPriority(acquireControllerCamera.getPriority() - 1);
        acquireControllerCamera.setDaemon(true);
        acquireControllerCamera.start();
        vision = true;
      }
      else
      {
        DriverStation.reportWarning("No ControllerVision process", false);
        System.out.println("Make sure camera is plugged in" +
          (Robot.isSimulation() ? " and number is correct (try the other one)" : ""));

        controllerVision = null;
      }

      break;

    case usePhotonVision:
      /*
      Select the camera name

      Enter the camera location wrt the robot. Example camera mounted facing
      forward (zero radians yaw), half meter forward of center, quarter meter up from center,
      zero meters left of center, pitch pointing up 25 degrees. (camera pointing up is "-""; down is "+"")
      */

      var cameraName = "Arducam_OV9281_USB_Camera"; // "Arducam_OV9281_USB_Camera"
      // Camera is centered on front of robot, 3.6875" forward from center, 2" above center, mounted flat
      var robotToCameraPV = new Transform3d(new Translation3d(Units.inchesToMeters(3.6875), 0.0, Units.inchesToMeters(2.0)),
                new Rotation3d(0, 0, 0));
      robotToCamera = robotToCameraPV;

      // PV started connecting when the NT server started but still it's a race to make
      // a connection before this instantiation and check
      try {
        Thread.sleep(3000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }

      photonVision = new PhotonVision(cameraName, robotToCameraPV);

      final var retryDelay = Seconds.of(1);
      final var retryLimit = 20;
      
      checkConnection:
      {
      for (int i = 1; i <= retryLimit; i++)
      {
        if (photonVision.camera.isConnected())
        {
          vision = true;
          break checkConnection;
        }
        System.out.println("attempt " + i + " of " + retryLimit + " to attach to PhotonVision named " + cameraName);      
        try {
          Thread.sleep((long)retryDelay.in(Milliseconds));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }

      photonVision.camera.getAllUnreadResults(); // force PV to issue no camera by that name but there are other PV cameras

      if (Robot.isSimulation())
      {
        System.out.println("Enter in the PV Settings Team Number/NetworkTables Server Address the IP address of this simulation device on the interface to the PhotonVision device");
        try {
          System.out.println("If PV running on the same device as this simulation, then use any of the IP addresses assign to that device such as 127.0.0.1, localhost, or " +
              InetAddress.getLocalHost().getHostAddress());
        } catch (UnknownHostException e) {
          e.printStackTrace();
        }
        System.out.println(RobotContainer.whatsMyIPAddress());
      }
      else // it's real roboRIO
      {
        System.out.println("Make sure the team number " + RobotController.getTeamNumber() + " is entered to attach to NetworkTables");
      }

      DriverStation.reportWarning("No PhotonVision connection", false);
     
      photonVision = null; // put it back to null since constructor didn't really make a viable camera
      }

      break;

    case useLimelightVision:
      /*
      Caution: Limelight pitch setup has the opposite sign of PhotonVision and ControllerVision
      LL positive pitch is pointing up
      Snap Robot to Floor is only for MegaTag; MegaTag2 always snaps to floor.
      Further than about 1.1m away from tag, MegaTag tends to have a lot of jitter especially in
      the z (height) axis.

      Add additional Limelight setups as indicated below
      */

      // Select the limelight name that is set in the Limelight box through its dashboard
      var limelightName = "limelight";
      if (LimelightVision.isAvailable(limelightName))
      {
        limelightVision = new LimelightVision(limelightName, RobotContainer.getDrivetrain());
        var c = LimelightHelpers.getCameraPose3d_RobotSpace(limelightName); // retrieve camera position in LL
        var robotToCameraLL = new Transform3d(c.getTranslation(),
              new Rotation3d(c.getRotation().getX(), -c.getRotation().getY(), c.getRotation().getZ())); // - pitch to match CV & PV
        robotToCamera = robotToCameraLL;
        vision = true;
        // put additional LL setup as needed
        // LimelightHelpers.setStreamMode_PiPSecondary(limelightName); // if there is a driver camera, where to put the image - pick your favorite
      }
      else
      {
        DriverStation.reportWarning("No LimelightVision connection", false);

        limelightVision = null;
      }

     break;
    
    default:
        System.out.println("impossible unless the enum is wrong");
        break;
    }
  }

  /**
   * 
   * @return if there is a Vision System active
   */
  public boolean vision()
  {
    return vision;
  }

  /**
   * periodically executed to update vision information
   */
  public void update()
  {
    if ( ! vision())
    {
      return;
    }

    switch (visionSelector)
    {
    case useControllerVision:
        controllerVision.update();
        if (controllerVision.isFresh())
        {
          robotPose.set(Optional.of(new RobotPose(
            controllerVision.getTagID(),
            controllerVision.getTX(), controllerVision.getTY(),
            controllerVision.getPose3d(), 
            controllerVision.getCameraToTarget())));
        }
        else
        {
          robotPose.set(Optional.empty());
        }

        break;

    case usePhotonVision:
        photonVision.update();
        if (photonVision.isFresh())
        {
          robotPose.set(Optional.of(new RobotPose(
            photonVision.getTagID(),
            photonVision.getTX(), photonVision.getTY(),
            photonVision.getPose3d(), 
            photonVision.getCameraToTarget())));
          // System.out.println("PV " + photonVision.getTagID() + ", " + photonVision.getTX() + ", " + photonVision.getTY() + ", " + photonVision.getPose3d());          
        }
        else
        {
          robotPose.set(Optional.empty());
        }

        break;

    case useLimelightVision:
        limelightVision.update();
        if (limelightVision.isFresh())
        {
          robotPose.set(Optional.of(new RobotPose(
            limelightVision.getTagID(),
            limelightVision.getTX(), limelightVision.getTY(),
            limelightVision.getPose3d(),
            limelightVision.getCameraToTarget())));
          // System.out.println((limelightVision.getSuggestResetOdometry() ?
          //   "reset odometry pose " : "addVisionMeasurement pose ") +
          //    limelightVision.getPose2d() + (limelightVision.isMegaTag2() ? " MegaTag2 pose" : " MegaTag pose"));
        }
        else
        {
          robotPose.set(Optional.empty());
        }

        break;

    default:
        System.out.println("impossible unless the enum is wrong");
        break;
    }
    
    // getRobotPose().ifPresentOrElse
    //   (
    //   (pose)-> {
    //       var t = getRobotPose().get().cameraToTarget;
    //       System.out.println("tag is forward " + t.getX() + "m, left " + t.getY() + "m, up " + t.getZ() +
    //          "m, roll " + Units.radiansToDegrees(t.getRotation().getX()) +
    //          "deg, pitch " + Units.radiansToDegrees(t.getRotation().getY()) +
    //          "deg, yaw " + Units.radiansToDegrees(t.getRotation().getZ()) + "deg");},
    //   ()-> System.out.print("X")
    //   );
  }

  /**
   * Robot pose for whomever wants it
   * @return RobotPose
   */
  public Optional<RobotPose> getRobotPose()
  {
      return robotPose.get();
  }

  /**
   * 
   * @return location of the camera wrt the robot
   */
  public Transform3d getRobotToCamera()
  {
      return robotToCamera;
  }

  /**
   * Getter for vision system for CameraBase getters
   * @return
   */
  public PhotonVision getPhotonVision() {
    return photonVision;
  }

  /**
   * Getter for vision system for CameraBase getters
   * @return
   */
  public LimelightVision getLimelightVision() {
      return limelightVision;
  }

  /**
   * Getter for vision system for CameraBase getters
   * @return
   */
  public ControllerVision getControllerVision() {
      return controllerVision;
  }
}
