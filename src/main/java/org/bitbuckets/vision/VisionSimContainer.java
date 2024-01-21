package org.bitbuckets.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.bitbuckets.drive.OdometrySubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSimContainer {
    final VisionSystemSim visionSystemSim;
    final VisionSubsystem visionSubsystem;
    final OdometrySubsystem odometrySubsystem;

    final AprilTagFieldLayout layout;
    public VisionSimContainer(VisionSubsystem visionSubsystem, OdometrySubsystem odometrySubsystem,
                              PhotonCamera camera_1, PhotonCamera camera_2, AprilTagFieldLayout layout) {
        this.visionSubsystem = visionSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.layout = layout;

        this.visionSystemSim = new VisionSystemSim("main");

        visionSystemSim.addAprilTags(layout);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 800, new Rotation2d(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture frame rate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCameraSim camera1Sim = new PhotonCameraSim(camera_1, cameraProp);
        PhotonCameraSim camera2Sim = new PhotonCameraSim(camera_2, cameraProp);

        visionSystemSim.addCamera(camera1Sim,new Transform3d(0, 0, 0,
                new Rotation3d(0, 0, 0)));

        visionSystemSim.addCamera(camera2Sim,new Transform3d(0, 0, 0,
                new Rotation3d(0, 0, 0)));

        camera1Sim.enableDrawWireframe(true);
        camera1Sim.enableProcessedStream(true);
        // camera2Sim.enableDrawWireframe(false);
        // camera2Sim.enableProcessedStream(false);
    }

    public void simulationPeriodic() {
        System.out.println(odometrySubsystem.getCurrentPosition());
        visionSystemSim.update(odometrySubsystem.getCurrentPosition());
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
