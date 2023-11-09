package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class IMU {

    public IMU() {
    }

    public abstract void zeroGyro();

    public abstract void offsetGyro(double angle);

    public abstract Rotation2d getYaw();

    public abstract Rotation2d getPitch();

    public abstract Rotation2d getRoll();
}