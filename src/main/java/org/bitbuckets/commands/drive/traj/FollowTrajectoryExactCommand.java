package org.bitbuckets.commands.drive.traj;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.bitbuckets.drive.AutoSubsystem;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;

import java.util.Optional;

/**
 * Follows a trajectory. Doesn't stop the motors once it ends and ends when the calculated trajectory time is over, doesn't leave room for PID controllers to finish.
 * You probably dont want to use this one directly
 */
public class FollowTrajectoryExactCommand extends Command {


    final Timer timer = new Timer();

    final ChoreoTrajectory trajectory;
    final OdometrySubsystem odometrySubsystem;
    final DriveSubsystem driveSubsystem;

    final PIDController xPid;
    final PIDController yPid;
    final ProfiledPIDController thetaPid;

    public FollowTrajectoryExactCommand(ChoreoTrajectory trajectory, OdometrySubsystem odometrySubsystem, DriveSubsystem driveSubsystem, PIDController xPid, PIDController yPid, ProfiledPIDController thetaPid) {
        this.trajectory = trajectory;
        this.odometrySubsystem = odometrySubsystem;
        this.driveSubsystem = driveSubsystem;
        this.autoSubsystem = autoSubsystem;

        this.xPid = xPid;
        this.yPid = yPid;
        this.thetaPid = thetaPid;
    }

    boolean shouldMirror() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElseThrow();
        return alliance == DriverStation.Alliance.Red;
    }

    @Override
    public void initialize() {
        thetaPid.reset(odometrySubsystem.getGyroAngle().getRadians());
        thetaPid.enableContinuousInput(-Math.PI, Math.PI);
        thetaPid.setTolerance(Math.PI / 360 ); //0.5 deg
        timer.restart();
    }

    @Override
    public void execute() {

        double time = timer.get();
        ChoreoTrajectoryState trajectoryReference = trajectory.sample(time, shouldMirror());
        Pose2d robotState = odometrySubsystem.getRobotCentroidPosition();
        ChassisSpeeds velocityFeedback = autoSubsystem.calculateFeedbackSpeeds(trajectoryReference.getPose());

        double xFF = trajectoryReference.velocityX;
        double yFF = trajectoryReference.velocityY;
        double rotationFF = trajectoryReference.angularVelocity;

        double xFeedback = xPid.calculate(robotState.getX(), trajectoryReference.x);
        double yFeedback = yPid.calculate(robotState.getY(), trajectoryReference.y);
        double rotationFeedback = thetaPid.calculate(odometrySubsystem.getGyroAngle().getRadians(), trajectoryReference.heading);

        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                trajectoryReference.velocityX + velocityFeedback.vxMetersPerSecond,
                trajectoryReference.velocityY + velocityFeedback.vyMetersPerSecond,
                trajectoryReference.angularVelocity + velocityFeedback.omegaRadiansPerSecond,
                robotState.getRotation()
        );



        driveSubsystem.driveUsingChassisSpeed(robotRelativeSpeeds, true);
    }

    @Override
    public boolean isFinished() {
         //
        return (timer.hasElapsed(trajectory.getTotalTime()) && autoSubsystem.isThetaAtSetpoint())
                || timer.hasElapsed(trajectory.getTotalTime() + 2);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (interrupted) {
            driveSubsystem.driveUsingChassisSpeed(new ChassisSpeeds(), false);
        }
    }
}
