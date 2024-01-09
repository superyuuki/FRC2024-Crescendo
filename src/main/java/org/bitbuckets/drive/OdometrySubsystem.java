package org.bitbuckets.drive;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.IO;
import org.bitbuckets.vision.VisionSubsystem;
import xyz.auriium.mattlib2.IPeriodicLooped;

import java.util.Optional;

public class OdometrySubsystem implements Subsystem, IPeriodicLooped {

    final DriveSubsystem driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final SwerveDrivePoseEstimator odometry;


    public OdometrySubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, SwerveDrivePoseEstimator odometry) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.odometry = odometry;
    }



    @Override
    public void periodic() {
        odometry.update(null,driveSubsystem.currentPositions());

        //VISION
        Optional<Pose3d> visionThinks = visionSubsystem.estimateVisionRobotPose_1();
        if (visionThinks.isPresent()) {
            Pose2d maybeAPose = visionThinks.get().toPose2d();

            odometry.addVisionMeasurement(maybeAPose, MathSharedStore.getTimestamp());
        }


    }

    @Override
    public void logPeriodic() {
        IO.SWERVE.logPosition(odometry.getEstimatedPosition());
    }
}