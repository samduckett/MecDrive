package frc.lib.util.gyrotypes;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.IMU;

public class Pigeon2 extends IMU {
    public PigeonIMU pigeon;
    public boolean invertGyro;

    public Pigeon2(int canID, boolean invertGyro) {
        invertGyro = this.invertGyro;
        pigeon = new PigeonIMU(canID);
        BasePigeonSimCollection pigeonSim = pigeon.getSimCollection();
    }

    public Pigeon2(int canID) {
        invertGyro = false;
        pigeon = new PigeonIMU(canID);
    }

    public void zeroGyro() {
        pigeon.setYaw(0);
    }

    public void offsetGyro(double angle) {
        pigeon.setYaw(angle);
    }

    public Rotation2d getYaw() {
        return invertGyro ? Rotation2d.fromDegrees(360 - pigeon.getYaw())
                : Rotation2d.fromDegrees(pigeon.getYaw());
    }

    public Rotation2d getPitch() {
        return invertGyro ? Rotation2d.fromDegrees(360 - pigeon.getPitch())
                : Rotation2d.fromDegrees(pigeon.getRoll());
    }

    public Rotation2d getRoll() {
        return invertGyro ? Rotation2d.fromDegrees(360 - pigeon.getRoll())
                : Rotation2d.fromDegrees(pigeon.getPitch());
    }

}
