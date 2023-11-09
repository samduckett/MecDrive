package frc.lib.util.gyrotypes;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.IMU;

public class pidgion extends IMU {
    public PigeonIMU gyro;
    public boolean invertGyro;

    public pidgion(int canID, boolean invertGyro) {
        invertGyro = this.invertGyro;
        gyro = new PigeonIMU(canID);
    }

    public pidgion(int canID) {
        invertGyro = false;
        gyro = new PigeonIMU(canID);
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public void offsetGyro(double angle) {
        gyro.setYaw(angle);
    }

    public Rotation2d getYaw() {
        return invertGyro ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getPitch() {
        return invertGyro ? Rotation2d.fromDegrees(360 - gyro.getPitch())
                : Rotation2d.fromDegrees(gyro.getRoll());
    }

    public Rotation2d getRoll() {
        return invertGyro ? Rotation2d.fromDegrees(360 - gyro.getRoll())
                : Rotation2d.fromDegrees(gyro.getPitch());
    }

}
