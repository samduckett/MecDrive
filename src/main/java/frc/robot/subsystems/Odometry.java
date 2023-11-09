package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;

public class Odometry {
    private MecanumDriveOdometry odometry;

    public Odometry(MecanumDriveKinematics kinematics, Rotation2d yaw, MecanumDriveWheelPositions whealPos,
            Pose2d startingPos) {
        odometry = new MecanumDriveOdometry(
                kinematics,
                yaw,
                whealPos,
                startingPos);
    }

    public void reset(Rotation2d yaw, MecanumDriveWheelPositions whealPos, Pose2d startingPos) {
        System.out.println("reset Odometry");
        odometry.resetPosition(yaw, whealPos, startingPos);
    }

    public void update(Rotation2d yaw, MecanumDriveWheelPositions pos) {
        odometry.update(yaw, pos);
    }

    public Pose2d get() {
        return odometry.getPoseMeters();
    }
}
