package org.bitbuckets;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.bitbuckets.commands.drive.DefaultDriveCommand;
import org.bitbuckets.commands.drive.MoveToAlignCommand;
import org.bitbuckets.commands.shooter.IntakeCommand;
import org.bitbuckets.commands.shooter.SetAmpShootingAngleCommand;
import org.bitbuckets.commands.shooter.SetSpeakerShootingAngleCommand;
import org.bitbuckets.commands.shooter.ShootNoteCommand;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.DrivebaseComponent;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.drive.SwerveModule;
import org.bitbuckets.shooter.ShooterSubsystem;
import org.bitbuckets.util.ThriftyEncoder;
import org.bitbuckets.util.Util;
import org.bitbuckets.vision.VisionComponent;
import org.bitbuckets.vision.VisionSubsystem;
import xyz.auriium.mattlib2.Mattlib;
import xyz.auriium.mattlib2.MattlibSettings;
import xyz.auriium.mattlib2.hardware.ILinearMotor;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;
import xyz.auriium.mattlib2.hardware.config.*;
import xyz.auriium.mattlib2.rev.HardwareREV;

import static xyz.auriium.mattlib2.Mattlib.LOG;

public class RobotContainer {


    //Components MUST be created in the Robot class (because of how static bs works)
    public static final VisionComponent VISION = LOG.load(VisionComponent.class, "vision");
    public static final DrivebaseComponent DRIVE = LOG.load(DrivebaseComponent.class, "swerve");

    public static final CommonMotorComponent DRIVE_COMMON = LOG.load(CommonMotorComponent.class, "swerve/drive_common");
    public static final CommonMotorComponent STEER_COMMON = LOG.load(CommonMotorComponent.class, "swerve/steer_common");
    public static final CommonPIDComponent PID_COMMON = LOG.load(CommonPIDComponent.class, "swerve/steer_pid_common");

    public static final MotorComponent[] DRIVES = MotorComponent.ofRange(DRIVE_COMMON, LOG.loadRange(IndividualMotorComponent.class, "swerve/drive", 4, Util.RENAMER));
    public static final MotorComponent[] STEERS = MotorComponent.ofRange(STEER_COMMON, LOG.loadRange(IndividualMotorComponent.class, "swerve/steer", 4, Util.RENAMER));
    public static final PIDComponent[] PIDS = PIDComponent.ofRange(PID_COMMON, LOG.loadRange(IndividualPIDComponent.class, "swerve/pid", 4, Util.RENAMER));

    public static final PIDComponent DRIVE_X_PID = LOG.load(PIDComponent.class, "swerve/x_holonomic_pid");
    public static final PIDComponent DRIVE_Y_PID = LOG.load(PIDComponent.class, "swerve/y_holonomic_pid");
    public static final PIDComponent DRIVE_T_PID = LOG.load(PIDComponent.class, "swerve/t_holonomic_pid");


    public final DriveSubsystem driveSubsystem;
    public final OperatorInput operatorInput;
    public final ShooterSubsystem shooterSubsystem;
    public final OdometrySubsystem odometrySubsystem;
    public final VisionSubsystem visionSubsystem;

    public RobotContainer() {
        CommandScheduler.getInstance().enable();

        //mattlib stuff
        Mattlib.LOOPER.runPreInit();
        Mattlib.LOOPER.runPostInit();
        MattlibSettings.USE_LOGGING = true;

        this.operatorInput = new OperatorInput();
        this.driveSubsystem = loadDriveSubsystem();
        this.shooterSubsystem = loadShooterSubsystem();
        this.odometrySubsystem = loadOdometrySubsystem();
        this.visionSubsystem = loadVisionSubsystem();

        loadCommands();
    }

    void loadCommands() {
        DefaultDriveCommand defaultDriveCommand = new DefaultDriveCommand(driveSubsystem, operatorInput);

        //When driver
        Trigger xGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1);
        Trigger yGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1);
        Trigger rotGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kRightX.value, 0.1);

        operatorInput.isTeleop.and(xGreaterThan.or(yGreaterThan).or(rotGreaterThan)).whileTrue(defaultDriveCommand);

        driveSubsystem.setDefaultCommand(new DefaultDriveCommand(driveSubsystem, operatorInput));
        operatorInput.ampSetpoint_hold.whileTrue(new SetAmpShootingAngleCommand(shooterSubsystem));
        operatorInput.speakerSetpoint_hold.whileTrue(new SetSpeakerShootingAngleCommand(shooterSubsystem));
        // .andThen(new ShootNoteCommand(shooterSubsystem))
        operatorInput.shootManually.onTrue(new ShootNoteCommand(shooterSubsystem));
        operatorInput.sourceIntake_hold.whileTrue(new IntakeCommand(shooterSubsystem));
        operatorInput.setShooterAngleManually.onTrue(new SetShootingAngleManuallyCommand(operatorInput, shooterSubsystem));

        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
                new PIDController(DRIVE_X_PID.pConstant(),DRIVE_X_PID.iConstant(),DRIVE_X_PID.dConstant()),
                new PIDController(DRIVE_Y_PID.pConstant(), DRIVE_Y_PID.iConstant(), DRIVE_Y_PID.dConstant()),
                new ProfiledPIDController(DRIVE_T_PID.pConstant(), DRIVE_T_PID.iConstant(), DRIVE_T_PID.dConstant(),
                        new TrapezoidProfile.Constraints(1,2)) //TODO
        );

        operatorInput.autoAlignHold.whileTrue(new MoveToAlignCommand(driveSubsystem, visionSubsystem, odometrySubsystem, holonomicDriveController));



    }

    DriveSubsystem loadDriveSubsystem() {
        SwerveModule[] modules = loadSwerveModules();
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d[4]);
        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(DRIVE.ff_ks(), DRIVE.ff_kv(), DRIVE.ff_ka());
        return new DriveSubsystem(modules,kinematics,ff);
    }

    SwerveModule[] loadSwerveModules() {
        SwerveModule[] modules = new SwerveModule[4];
        for (int i = 0; i < modules.length; i++) {
            ILinearMotor driveMotor;
            IRotationalController steerController;
            IRotationEncoder absoluteEncoder;
            if (Robot.isSimulation()) {
                driveMotor = null;
                steerController = null;
                absoluteEncoder = null;
            } else {
                driveMotor = HardwareREV.linearSpark_noPID(DRIVES[i]);
                steerController = HardwareREV.rotationalSpark_builtInPID(STEERS[i], PIDS[i]);
                absoluteEncoder = new ThriftyEncoder();
            }
            modules[i] = new SwerveModule(driveMotor, steerController, absoluteEncoder);
        }
        return modules;
    }

    ShooterSubsystem loadShooterSubsystem() {
        return null; //TODO
    }
    OdometrySubsystem loadOdometrySubsystem() {
        return null; //TODO
    }
    VisionSubsystem loadVisionSubsystem() {
        return null; //TODO
    }
}
