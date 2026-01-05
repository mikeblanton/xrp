package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * PhotonVision requires significant setup using the dashboard and, somewhat, code can be used.
 * 
 * <p>In addition to what PhotonVision publishes to NetworkTables this program publishes robot
 * Pose3d and yaw and pitch for each tag detected to table "PhotonVisionLogged".
 * 
 * <p>PV always returns a 3-D pose even if it not activated by calibrating the camera and selecting
 * 3-D Processing Mode on the PV dashboard. The 3-D pose is made zero (origin) for this mode. This
 * value may occasionally conflict with a "legitimate" (calculated?) zero pose but that's not suppose
 * to happen as there is a corner of the field there where the center of a robot can't go.
 * {@link #getPose3d()}.
 */
public class PhotonVision extends CameraBase {
    static
    {
      System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    PhotonCamera camera;
    private boolean targetVisible;
    private PhotonTrackedTarget bestTarget;
    private Transform3d cameraInRobotFrame;

    private int maxTagId = 25    +1; // add 1 for tag ID 0
    private NetworkTable RobotPoseTable = NetworkTableInstance.getDefault().getTable("PhotonVisionLogged");
    
    private List<StructPublisher<Pose3d>> publishRobotPose = new ArrayList<>(maxTagId); // Pose3d
    private List<DoubleArrayPublisher> publishRobotServoing = new ArrayList<>(maxTagId); // yaw and pitch
    
    // PhotonPipelineResult PVResult = new PhotonPipelineResult();

    public PhotonVision(String name, Transform3d cameraInRobotFrame) {
      this.cameraInRobotFrame = cameraInRobotFrame;
      camera = new PhotonCamera(name);

      // make an empty bucket for every possible tag
      for (int tag = 0; tag < maxTagId; tag++) {
        var robotPosePublisher = RobotPoseTable.getStructTopic("robotPose3D_" + tag, Pose3d.struct).publish();
        publishRobotPose.add(robotPosePublisher);

        var robotServoingPublisher = RobotPoseTable.getDoubleArrayTopic("robotServoing_" + tag).publish();
        publishRobotServoing.add(robotServoingPublisher);
      }

      // better poses possible using the PhotonPoseEstimator; lot's of possibilities with it
      // PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameraInRobotFrame);
    }

    /**
     * Read relevant data from the Camera
     */
    public void update() {

    targetVisible = false;
    var results = camera.getAllUnreadResults();

    if ( ! results.isEmpty()) {
        var result = results.get(results.size() - 1); // Get the last, most current result in the list.
        if (result.hasTargets()) { // At least one AprilTag was seen by the camera

            // example looping through all targets seen in this frame. An individual tag information
            // could be pulled off here as the PhotonVision example program shows.
            // var countTargets = 0;
            // for (var target : result.getTargets()) {
            //   System.out.println("target " + ++countTargets + " " + target);
            // }

            // for this example use the PV designated Best Target
            bestTarget = result.getBestTarget();
            // double area = bestTarget.getArea();
            // double skew = bestTarget.getSkew();
            // List<TargetCorner> corners = bestTarget.getDetectedCorners();
            // double poseAmbiguity = bestTarget.getPoseAmbiguity();
            // Transform3d alternateCameraToTarget = bestTarget.getAlternateCameraToTarget();

            // https://docs.photonvision.org/en/v2025.0.0-beta-4/docs/apriltag-pipelines/multitag.html
             //Enabling MultiTag

            // if (result.getMultiTagResult().isPresent()) {
            //   Transform3d fieldToCamera = result.getMultiTagResult().get().estimatedPose.best;
            // }

            // Correct pose estimate with vision measurements
        // var visionEst = vision.getEstimatedGlobalPose();
        // visionEst.ifPresent(
        //         est -> {
        //             // Change our trust in the measurement based on the tags we can see
        //             var estStdDevs = vision.getEstimationStdDevs();

        //             drivetrain.addVisionMeasurement(
        //                     est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        //         });

            // System.out.println(/*"\nRESULT " + result +*/ "\n\nPV BEST TARGET " + bestTarget);
            var tag = bestTarget.getFiducialId();
            publishRobotPose.get(tag).set(getPose3d());
            publishRobotServoing.get(tag).set(new double[]{getTX(), getTY()});

            targetVisible = true;
        }
        else {
        }
      }
    }

    @Override
    public boolean isFresh() {
      return targetVisible;
    }

    @Override
    public Pose3d getPose3d() {
      var tagInCameraFrame = getCameraToTarget();
      if (tagInCameraFrame.equals(Transform3d.kZero))
      {
        return Pose3d.kZero;
      }
      else
      {
        Pose3d tagInFieldFrame = AprilTagsLocations.getTagLocation(getTagID());
        var robotInFieldFrame = ComputerVisionUtil.objectToRobotPose(tagInFieldFrame,  tagInCameraFrame,  cameraInRobotFrame);
        return robotInFieldFrame;
      }
    }

    @Override
    public Pose2d getPose2d() {
        return new Pose2d(getPose3d().getX(), getPose3d().getY(), new Rotation2d(getPose3d().getRotation().getZ()));
    }

    @Override
    public double getTX() {
      return bestTarget.getYaw();
    }

    @Override
    public double getTY() {
      return bestTarget.getPitch();
    }

    @Override
    public int getTagID() {
      return bestTarget.getFiducialId();
    }

    public Transform3d getCameraToTarget()
    {
      return bestTarget.getBestCameraToTarget();
    }
    
    /**
     * call this then use it in addVisionMeasurement
     * 
     * <p> https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
     * 
     * <p>It's complicated but this is a hint to get you started
     * @param prevEstimatedRobotPose
     * @return
     */
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    //     return photonPoseEstimator.update(/*dummy cameraResult to compile*/new PhotonPipelineResult());
    // }
}
