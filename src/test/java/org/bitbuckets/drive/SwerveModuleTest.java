package org.bitbuckets.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;
import xyz.auriium.mattlib2.hardware.ILinearMotor;
import xyz.auriium.mattlib2.hardware.IRotationEncoder;
import xyz.auriium.mattlib2.hardware.IRotationalController;

import static org.bitbuckets.RobotContainer.SWERVE;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

public class SwerveModuleTest {

    ILinearMotor driveMotor;
    IRotationalController steerController;
    IRotationEncoder absoluteEncoder;
    SimpleMotorFeedforward ff;

    @BeforeEach
    public void beforeEach() {
        driveMotor = mock(ILinearMotor.class);
        steerController = mock(IRotationalController.class);
        absoluteEncoder = mock(IRotationEncoder.class);
        ff = new SimpleMotorFeedforward(SWERVE.ff_ks(), SWERVE.ff_kv());

    }

    @Test
    public void testLogicPeriodic() {
        when(steerController.angularVelocity_mechanismRotationsPerSecond()).thenReturn(20d);
        when(absoluteEncoder.angularPosition_normalizedMechanismRotations()).thenReturn(10d);

        double absoluteAngularPosition_infiniteMechanismRotations = absoluteEncoder.angularPosition_normalizedMechanismRotations();
        steerController.forceRotationalOffset(absoluteAngularPosition_infiniteMechanismRotations);

        assertEquals(0d, steerController.angularPosition_mechanismRotations(), 0);
    }

    @Test
    public void testSetToMoveAtStraight() {
        SwerveModule module = new SwerveModule(driveMotor, steerController, absoluteEncoder, ff);

        module.setToMoveAt(new SwerveModuleState(20, Rotation2d.fromDegrees(0)));
        verify(driveMotor).setToVoltage(12);
        verify(steerController).controlToNormalizedReference(0);
    }

    @Test
    public void testSetToMoveAtTurn1() {
        SwerveModule module = new SwerveModule(driveMotor, steerController, absoluteEncoder, ff);

        // move a quarter
        module.setToMoveAt(new SwerveModuleState(20, Rotation2d.fromDegrees(90)));
        verify(steerController).controlToNormalizedReference(.25);
    }

    @Test
    public void testSetToMoveAtTurn2() {
        SwerveModule module = new SwerveModule(driveMotor, steerController, absoluteEncoder, ff);

        // move a quarter
        module.setToMoveAt(new SwerveModuleState(20, Rotation2d.fromDegrees(-90)));
        verify(steerController).controlToNormalizedReference(.75);
    }

    @Test
    public void testSetToMoveAtTurn3() {
        SwerveModule module = new SwerveModule(driveMotor, steerController, absoluteEncoder, ff);

        // abs encoder registers 1/2 turn clockwise
        when(steerController.angularPosition_normalizedMechanismRotations()).thenReturn(.5);

        // rotate a bit counter-clockwise
        // the "asked for" rotation would be -.7, but we flip the drive motor in reverse and only
        // actually rotate .3
        module.setToMoveAt(new SwerveModuleState(20, Rotation2d.fromDegrees(-72)));

        // the drive motor should move backwards, and the mechanism should be set to .4 (-.1 + .5)
        verify(driveMotor).setToVoltage(-12);
        verify(steerController).controlToNormalizedReference(AdditionalMatchers.eq(.3, .01));
    }

    @Test
    public void testSetToMoveAtTurn4() {
        SwerveModule module = new SwerveModule(driveMotor, steerController, absoluteEncoder, ff);

        // abs encoder registers 1/4 turn clockwise
        when(steerController.angularPosition_normalizedMechanismRotations()).thenReturn(.25);

        // rotate a bit counter-clockwise
        module.setToMoveAt(new SwerveModuleState(20, Rotation2d.fromDegrees(-36)));

        // the drive motor should move backwards, and the mechanism should be set to .4 (-.1 + .5)
        verify(driveMotor).setToVoltage(-12);
        verify(steerController).controlToNormalizedReference(.4);
    }
}
