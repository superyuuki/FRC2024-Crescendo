package org.bitbuckets.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.drive.Odometry;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class VisionSimContainer {
    final VisionSystemSim visionSystemSim;
//    final VisionTargetSim visionTargets;
    final VisionSubsystem visionSubsystem;
    final Odometry odometry;

    final AprilTagFieldLayout layout;
    public VisionSimContainer(VisionSubsystem visionSubsystem, Odometry odometry,
                              PhotonCamera camera_1, PhotonCamera camera_2, AprilTagFieldLayout layout) {
        this.visionSubsystem = visionSubsystem;
        this.odometry = odometry;
        this.layout = layout;

//        TargetModel targetModel = new TargetModel(Units.inchesToMeters(6.5), Units.inchesToMeters(6.5));
//        Pose3d targetPose = new Pose3d(new Translation3d(652.73, 196.17, 57.13), new Rotation3d(0, 0, Math.PI));
//
//        this.visionTargets = new VisionTargetSim(targetPose, targetModel);

        this.visionSystemSim = new VisionSystemSim("main");

        visionSystemSim.addAprilTags(layout);
        // visionSystemSim.addVisionTargets(visionTargets);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture frame rate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(20);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCameraSim camera1Sim = new PhotonCameraSim(camera_1, cameraProp, 0, 4);
        PhotonCameraSim camera2Sim = new PhotonCameraSim(camera_2, cameraProp, 0, 4);

        visionSystemSim.addCamera(camera1Sim, new Transform3d(RobotContainer.CAMERAS.camera1TranslationOffset(), RobotContainer.CAMERAS.camera1RotationOffset()));

        visionSystemSim.addCamera(camera2Sim, new Transform3d(RobotContainer.CAMERAS.camera2TranslationOffset(), RobotContainer.CAMERAS.camera2RotationOffset()));

        camera1Sim.enableDrawWireframe(true);
        camera1Sim.enableProcessedStream(true);
        // camera2Sim.enableDrawWireframe(false);
        // camera2Sim.enableProcessedStream(false);
    }

    public void simulationPeriodic() {
        visionSystemSim.update(odometry.getRobotCentroidPosition());


        // visionSystemSim.update(new Pose2d(0, 0, new Rotation2d(0)));
        // Field2d debugField = visionSystemSim.getDebugField();
        // debugField.getObject("EstimatedRobot").setPose(odometrySubsystem.getCurrentPosition());
        // debugField.getObject("EstimatedRobot").setPose(new Pose2d(0, 0, new Rotation2d(0)));

//        Pose2d[] modulePoses = new Pose2d[swerveMods.length];
//        Pose2d swervePose = odometrySubsystem.getCurrentPosition();
//        for (int i = 0; i < swerveMods.length; i++) {
//            SwerveModule module = swerveMods[i];
//            SwerveModulePosition modPosition = module.getPosition();
//            modulePoses[i] = swervePose.transformBy(new Transform2d(
//                                            modPosition.distanceMeters, modPosition.angle));
//        }
//        return modulePoses;

        // debugField.getObject("EstimatedRobotModules").setPoses();


    }
}
