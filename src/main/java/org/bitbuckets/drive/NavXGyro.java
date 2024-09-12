package org.bitbuckets.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.CircularBuffer;
import xyz.auriium.mattlib2.loop.IMattlibHooked;

public class NavXGyro implements IGyro, IMattlibHooked {

    final AHRS ahrs;
    final int samples = 50;
    public NavXGyro(AHRS ahrs) {
        this.ahrs = ahrs;

        mattRegister();
    }

    @Override
    public void logicPeriodic() {
        angleBuffer.addFirst(ahrs.getAngle());
    }

    final CircularBuffer<Double> angleBuffer = new CircularBuffer<>(samples);
    Rotation2d tare = new Rotation2d();

    @Override
    public Rotation2d initializationRelativeRotation() {
        return ahrs.getRotation2d();
    }

    @Override
    public Rotation2d userZeroRelativeRotation() {
        return ahrs.getRotation2d().minus(tare);
    }

    @Override
    public void userZero() {
        tare = tare.plus(userZeroRelativeRotation());
    }

    @Override
    public void userForceOffset(Rotation2d beAt) {
        tare = tare.plus(userZeroRelativeRotation()).plus(beAt);
    }

    @Override
    public boolean isOk() {
        double average = 0d;
        for (int i = 0; i < samples; i++) {
            average += angleBuffer.get(i);
        }
        average /= samples;

        return (average != 0.0);
    }

}
