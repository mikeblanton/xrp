// Detecting and drawing AprilTags is significantly based on a WPILib example vision project so
// for ControllerVision.java, AcquireAprilTag.java, and AcquireRobotPose.java much is:
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.opencv.core.MatOfDouble;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * This is a demo program showing the detection of AprilTags and robot pose calculation using WPILib
 * functions.
 * <p>The image is acquired from the USB camera, then any detected AprilTags are marked up on the image,
 * transformed to the robot on the field pose and sent to NetworkTables.
 * 
 * <p>To remove irritating vertical (Z height) jitter an example of clamping the robot pose to the floor is
 * included in {@link AcquireRobotPose#run()}. That could be removed in AcquireRobotPose. (LimelightVision
 * setup has that option as set in the Limelight.)
 * 
 * <p>Especially at distances > about 1.1 meters there is side-to-side and forward-backward jitter
 * especially when the camera is straight-on to the tag and there is ambiguity on which side of the tag
 * the camera is on. A smoothing or simple averaging of two (or more?) successive poses may yield a
 * better result. Don't smooth out the fact that the robot might actually be moving.
 * 
 * <p>The camera image whiteness (contrast, gamma, brightness, exposure, gain) and sharpness have a
 * huge effect on jitter. These settings all interact so experiment. Often an image slightly
 * darker than what humans may find comfortable is about right. Try high exposure, high brightness,
 * low gain, low gamma, high contrast. Too bad high exposure is the opposite of what is needed for
 * high frame rate.
 * 
 * <p>Included is an example Spike Filter that does give improved performance for both X and Y
 * axes. See {@link AcquireRobotPose} to remove or tune them better for another environment.
 * 
 * <p>AcquireRobotPose has no provision for using the gyro to improve pose estimation. The gyro
 * heading could be used instead of the pose rotation, if desired.
 *
 * <p>Be aware that the performance on a roboRIO (especially v1) is much worse than a coprocessor solution!
 * 
 * <p>The camera view is displayed with additional information of the AprilTag pose to camera.
 * That display is optional and a tiny bit of cpu processing can be saved by not doing it.
 * Since AprilTag view can be in normal light, that camera can also be used by the operator if it's
 * pointing in a good direction. If the operator doesn't need that view, don't display it and
 * the image can be made a little darker and more contrast - whatever can reduce the cpu
 * processing time for an image. Experiment with exposure, contrast, gamma, brightness, etc.
 * 
 * <p>AprilTag has known pose on the field loaded from file (WPILib or your custom file).
 * Detected tag's perspective seen by the camera is used to calculate an estimate of the camera pose
 * relative to the tag.
 * Camera has a known pose relative to the robot chassis.
 * Combine this chain to calculate the robot pose in the field.
 * Camera parameters must be provided from another source such as a related calibration program.
 * 
 * <p>The original design of this vision system was {@link AcquireAprilTag} and {@link AcquireRobotPose}
 * ran in two separate free-wheeling threads that were synchronized as needed by {@link Image}. That
 * scheme has been disabled, however, some remnants remain. The implementation for this project is
 * this {@link ControllerVision} class runs in a separate free-wheeling thread started by a higher level
 * thread. Then this class {@link #run()} runs {@link AcquireAprilTag#run()} and
 * {@link AcquireRobotPose#run()} successively thus no synchronization is needed.
*/     
public class ControllerVision extends CameraBase implements Runnable {
  static
  {
      System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
  }

    public enum CameraOption{ArduCam320x240, ArduCam1280x800, LifeCam320x240, LifeCam640x480};

    int cameraDeviceId;

    // Calibrate the camera at the used resolution or scale Fx,Fy,Cx,Cy proportional
    // to what resolution was used for camera calibration. (But apparently recalibrating
    // at various resolutions does yield slightly varying results.)

    int cameraW; // image width
    int cameraH; // image height
    int fps;     // frames/second
    double cameraFx; // fx camera horizontal focal length, in pixels
    double cameraFy; // fy camera vertical focal length, in pixels
    double cameraCx; // cx camera horizontal focal center, in pixels
    double cameraCy; // cy camera vertical focal center, in pixels
    MatOfDouble distortionCoeffs;
    Image image = new Image(); // where a video frame goes for multiple processes to use
    private AcquireAprilTag acquireAprilTag;
    private AcquireRobotPose acquireRobotPose;
    private ArrayList<RobotPose> poses;
    private int bestPoseIndex;
    private RobotPose bestPose;

    public ControllerVision(int cameraDeviceId, CameraOption selectedCameraOption, boolean usePose3D, Transform3d robotToCamera)
    {
        this.cameraDeviceId = cameraDeviceId;

      switch(selectedCameraOption) {
        
        case ArduCam320x240:
          ArduCam320x240();
          break;

        case ArduCam1280x800:
          ArduCam1280x240();
          break;

        case LifeCam320x240:
          LifeCam320x240();
          break;

        case LifeCam640x480:
          LifeCam640x480();
          break;

        default:
          System.out.println("impossible unless the enum is wrong");
          break;
      }

      acquireAprilTag = new AcquireAprilTag(this);
      acquireRobotPose = new AcquireRobotPose(this, usePose3D, robotToCamera);
    }

    /**
     * See if camera by this device number is connected
     * <p>Sometimes linux/camera server says camera is connected when it isn't and then something else
     * often goes wrong.
     * @param cameraDeviceId
     * @return
     */
    public static boolean isAvailable(int cameraDeviceId)
    {
      for (UsbCameraInfo camera : UsbCamera.enumerateUsbCameras())
      {
        if (camera.dev == cameraDeviceId)
        {
            return true; // found camera so probably okay to try to start auto capture
        }
      }
      
      System.out.println("camera " + cameraDeviceId + " not found. Other cameras < ");
      for (UsbCameraInfo camera : UsbCamera.enumerateUsbCameras())
      {
        System.out.println(camera.dev + " " + camera.name + " " + camera.path);
      }
      System.out.println(" >");
      return false;
    }

    /**
     * Loop of the thread to acquire camera images and compute the poses of AprilTags
     */
    public void run()
    {
      // This 'while' cannot be 'true'. The program will never exit if it is. Using
      // interrupted() lets the robot stop this thread when restarting robot code or deploying.
      while ( ! Thread.interrupted()) {
        acquireAprilTag.run();
        acquireRobotPose.run();
      }
    }

    /**
     * Get latest data - must be first before accessing data
     */
    @Override
    public void update() {
        poses = acquireRobotPose.getPoses();
        // poses.forEach(pose -> System.out.println("CV " + pose)); // show all available data

        if (isFresh())
        {
          //FIXME determine best target in the list; dummy filter - arbitrarily pick the lowest number; this needs work
          bestPoseIndex = 0;
          bestPose = getPoses().get(bestPoseIndex).clone();
        }
    }

    /**
     * check for new data 
     * @return true if new data; false if stale or no data
     */
    @Override
    public boolean isFresh() {
        return poses != null;
    }

    // These are single-valued characteristics in a possibly multi-tag frame.
    // To see all tags detected use the getPoses() method and parse from it anything you want.

    /**
     * @return best 3-D pose
     */
    @Override
    public Pose3d getPose3d() {
      return bestPose.pose3D;
    }

    /**
     * @return best 2-D pose
     */
    @Override
    public Pose2d getPose2d() {
        return new Pose2d(bestPose.pose3D.getX(), bestPose.pose3D.getY(),
          new Rotation2d(bestPose.pose3D.getRotation().getZ()));
    }

    /**
     * @return best robot yaw to tag
     */
    @Override
    public double getTX() {
      return bestPose.yaw;
    }

    /**
     * @return best robot pitch to tag
     */
    @Override
    public double getTY() {
      return bestPose.pitch;
    }

    /**
     * @return best tag id
     */
    @Override
    public int getTagID() {
      return bestPose.AprilTagId;
    }

    /**
     * @return best transform from camera to target
     */
    @Override
    public Transform3d getCameraToTarget()
    {
      return bestPose.cameraToTarget;
    }

    /**
     * 
     * @return list of poses from all tag ids detected in the frame
     */
    public ArrayList<RobotPose> getPoses() {
      return poses;
    }

    private void ArduCam320x240()
    {
      // rough calibration - wasn't done with a nice flat board
      cameraW = 320;
      cameraH = 240;
      fps = 100;
      cameraFx = 273.8682279422785;
      cameraFy = 274.2578211409246;
      cameraCx = 142.187975375679;
      cameraCy = 124.6151823259089;
      distortionCoeffs = new MatOfDouble();
        // much of the small amount of distortion from calibration was actually the board
        // not being smooth so don't bother using the distortion
        // [0.03872533667096114, -0.2121025605447465, 0.00334472765894009, -0.006080540135581289, 0.4001779842036727]
    }

    private  void ArduCam1280x240()
    {
      // Arducam_OV9281_USB_Camera_(A?); Cameron board 1280x800      
      cameraW = 1280;
      cameraH = 800;
      fps = 100;
      cameraFx = 907.6920444758049;
      cameraFy = 907.1513951038395;
      cameraCx = 604.1750223777503;
      cameraCy = 416.4609913313957;
      distortionCoeffs = new MatOfDouble(
          0.040354289830866516, -0.044066115475547216, 6.662818829158613E-4,9.755603732755772E-4,  // k1 k2 p1 p2
            -0.013630390510289322, // k3
            -0.0011985508423857224, 0.003370423168524356, 0.0010337869630847195); // k4 k5 k6
          // assume s1 s2 s3 s4 tx ty are all zeros
    }

    private void LifeCam320x240()
    {
      // 320x240 lifecam calibration from PhotonVision
      cameraW = 320;
      cameraH = 240;
      fps = 30;
      cameraFx = 353.74653217742724;
      cameraFy = 340.77624878700817;
      cameraCx = 163.5540798921191;
      cameraCy = 119.8945718300403;
      distortionCoeffs = new MatOfDouble();
    }

    private void LifeCam640x480()
    {
      // 640x480 lifecam calibration from WPILib example
     // and https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21
      cameraW = 640;
      cameraH = 480;
      fps = 30;
      cameraFx = 699.3778103158814;
      cameraFy = 677.7161226393544;
      cameraCx = 345.6059345433618;
      cameraCy = 207.12741326228522;
      distortionCoeffs = new MatOfDouble();
    }
}
//opencv tutorial example
// https://www.wolframalpha.com/input/?i=plot (1 + -4.1802327176423804e-001 Power[\(40)Divide[r,6.5746697944293521e+002]\(41),2] + 5.0715244063187526e-001 Power[\(40)Divide[r,6.5746697944293521e+002]\(41),4] + -5.7843597214487474e-001*Power[\(40)Divide[r,6.5746697944293521e+002]\(41),6]) 
// all times r
// for r from 0 to 400
