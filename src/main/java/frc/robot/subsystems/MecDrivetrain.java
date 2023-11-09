package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MecDrivetrain extends DrivetrainBase {
    // Creating my kinematics object using the wheel locations.
    public MecanumDriveKinematics kinematics;

    /**
     * FL, FR, BL, BR
     */
    public MecModule[] mods = { new MecModule(0), new MecModule(1), new MecModule(2), new MecModule(3) };

    public MecDrivetrain() {
        super();

        imu = imu.new pidgion(1); // test
        kinematics = new MecanumDriveKinematics(
                wheelLocations[0], wheelLocations[1],
                wheelLocations[2], wheelLocations[3]);

        odometry = new Odometry(kinematics, getYaw(), getModsPos(), new Pose2d(0.0, 0.0, new Rotation2d()));

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

    public void setChassisSpeed(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        mods[0].driveMoter(wheelSpeeds.frontLeftMetersPerSecond);
        mods[1].driveMoter(wheelSpeeds.frontRightMetersPerSecond);
        mods[2].driveMoter(wheelSpeeds.rearLeftMetersPerSecond);
        mods[3].driveMoter(wheelSpeeds.rearRightMetersPerSecond);
    }

    public ChassisSpeeds getChassisSpeed() {
        return kinematics.toChassisSpeeds(getModsVel());
    }

    public MecanumDriveWheelPositions getModsPos() {
        return new MecanumDriveWheelPositions(mods[0].getPos(), mods[1].getPos(), mods[2].getPos(), mods[3].getPos());
    }

    public MecanumDriveWheelSpeeds getModsVel() {
        return new MecanumDriveWheelSpeeds(mods[0].getVel(), mods[1].getVel(), mods[2].getVel(), mods[3].getVel());
    }

    @Override
    public void periodic() {
        odometry.update(getYaw(), getModsPos());

        SmartDashboard.putNumber("Pitch", Math.abs(MathUtil.inputModulus(getPitch().getDegrees(), -180, 180)));

        for (MecModule mod : mods) {
            SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Cancoder", mod.getPosition());
            SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Velocity", mod.getVelocity());
        }
    }
}