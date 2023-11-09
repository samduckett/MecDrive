package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.IMU;

public abstract class DrivetrainBase extends SubsystemBase {
    public DrivetrainBase() {
        AutoBuilder.configureHolonomic(
                this::getPose2d, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        maxAngularVelocity, // Max module speed, in m/s
                        drivetrainRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                this // Reference to this subsystem to set requirements
        );

    }

    public double maxSpeed = 4.0;
    public double maxAngularVelocity = 2.0;
    public double maxAutoSpeed = 4.0;

    /** meaters from center fathest out point */
    public double drivetrainRadius = 0.4;

    public Odometry odometry;

    /**
     * Locations of the wheels relative to the robot center.
     * FL, FR, BL, BR
     */
    public Translation2d[] wheelLocations = {
            new Translation2d(0.381, 0.381),
            new Translation2d(0.381, -0.381),
            new Translation2d(-0.381, 0.381),
            new Translation2d(-0.381, -0.381)
    };

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        if (fieldRelative)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());
        setChassisSpeed(speeds);
    }

    public abstract void setChassisSpeed(ChassisSpeeds speeds);

    public abstract ChassisSpeeds getChassisSpeed();

    public abstract MecanumDriveWheelPositions getModsPos(); // FIX

    // Gyro------------------------------------------
    /** the can Id for the imu */
    public int IMUID = 1;

    /** the gyroscope for the chassie object */
    public IMU imu;

    public void zeroGyro() {
        imu.zeroGyro();
    }

    public void offsetGyro(double angle) {
        imu.offsetGyro(angle);
    }

    public Rotation2d getYaw() {
        return imu.getYaw();
    }

    public Rotation2d getPitch() {
        return imu.getPitch();
    }

    public Rotation2d getRoll() {
        return imu.getRoll();
    }

    // odometry-------------------------------------------
    public Pose2d getPose2d() {
        return odometry.get();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.reset(getYaw(), getModsPos(), pose);
    }
}
