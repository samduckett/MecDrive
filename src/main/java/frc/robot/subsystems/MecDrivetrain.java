package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.lib.math.Conversions;
import frc.lib.util.gyrotypes.*;;

public class MecDrivetrain extends DrivetrainBase {
    // Creating my kinematics object using the wheel locations.
    public MecanumDriveKinematics kinematics;

    /**
     * FL, FR, BL, BR
     */
    public MecModule[] moduels = new MecModule[4];

    public Field2d field;

    public MecDrivetrain() {
        /*
         * TalonFX m_leftDrive = new WPI_TalonFX(0);
         * TalonFX m_leftFollower = new WPI_TalonFX(1);
         * TalonFX m_rightDrive = new WPI_TalonFX(2);
         * TalonFX m_rightFollower = new WPI_TalonFX(3);
         * TalonFXSimCollection m_leftDriveSim = m_leftDrive.getSimCollection();
         * TalonFXSimCollection m_rightDriveSim = m_rightDrive.getSimCollection();
         * 
         * WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(1);
         * 
         * BasePigeonSimCollection m_pigeonSim = m_pigeon.getSimCollection();
         * final int kCountsPerRev = 4096; // Encoder counts per revolution of the motor
         * shaft.
         * final double kSensorGearRatio = 1; // Gear ratio is the ratio between the
         * encoder* and the wheels. On the
         * // AndyMark
         * // drivetrain, encoders mount 1:1 with the gearbox shaft.
         * final double kGearRatio = 10.71; // Switch kSensorGearRatio to this gear
         * ratio if encoder is on the motor
         * // instead
         * // of on the gearbox.
         * final double kWheelRadiusInches = 3;
         * final int k100msPerSecond = 10;
         * 
         * /* Simulation model of the drivetrain
         * 
         * // DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
         * // DCMotor.getCIM(2), // 2 CIMS on each side of the drivetrain.
         * // kGearRatio, // Standard AndyMark Gearing reduction.
         * // 2.1, // MOI of 2.1 kg m^2 (from CAD model).
         * // 26.5, // Mass of the robot is 26.5 kg.
         * // Units.inchesToMeters(kWheelRadiusInches), // Robot uses 3" radius (6"
         * // diameter) wheels.
         * // 0.546, // Distance between wheels is _ meters.
         * 
         * // /*
         * // * The standard deviations for measurement noise:
         * // * x and y: 0.001 m
         * // * heading: 0.001 rad
         * // * l and r velocity: 0.1 m/s
         * // * l and r position: 0.005 m
         * //
         * // null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
         * // //Uncomment this
         * // // line to add measurement noise.
         * // );
         * 
         * m_rightFollower.configFactoryDefault();
         * 
         * m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
         * m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
         * 
         * m_driveSim.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
         * -m_rightDriveSim.getMotorOutputLeadVoltage());
         * Conversions.MPSToFalcon(kWheelRadiusInches, k100msPerSecond, kGearRatio)
         * m_driveSim.update(0.02);
         * 
         * m_leftDriveSim.setQuadratureRawPosition(distanceToNativeUnits(m_driveSim.
         * getLeftPositionMeters()));
         * m_pigeonSim.setRawHeading(m_driveSim.getHeading().getDegrees());
         * -------------------------------------
         */
        moduels[0] = new MecModule(0);
        moduels[1] = new MecModule(1);
        moduels[2] = new MecModule(2);
        moduels[3] = new MecModule(3);

        imu = new Pigeon2(1); // test

        kinematics = new MecanumDriveKinematics(wheelLocations[0], wheelLocations[1], wheelLocations[2],
                wheelLocations[3]);

        odometry = new Odometry(kinematics, imu.getYaw(), getModsPos(), new Pose2d(0.0, 0.0, new Rotation2d()));

        field = new Field2d();

    }

    public void setChassisSpeed(ChassisSpeeds speeds) {
        SmartDashboard.putString("Chassie Speed", speeds.toString());
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        moduels[0].driveMoter(wheelSpeeds.frontLeftMetersPerSecond);
        moduels[1].driveMoter(wheelSpeeds.frontRightMetersPerSecond);
        moduels[2].driveMoter(wheelSpeeds.rearLeftMetersPerSecond);
        moduels[3].driveMoter(wheelSpeeds.rearRightMetersPerSecond);
    }

    public ChassisSpeeds getChassisSpeed() {
        return kinematics.toChassisSpeeds(getModsVel());
    }

    public MecanumDriveWheelPositions getModsPos() {
        return new MecanumDriveWheelPositions(moduels[0].getPos(), moduels[1].getPos(), moduels[2].getPos(),
                moduels[3].getPos());
    }

    public MecanumDriveWheelSpeeds getModsVel() {
        return new MecanumDriveWheelSpeeds(moduels[0].getVel(), moduels[1].getVel(), moduels[2].getVel(),
                moduels[3].getVel());
    }

    @Override
    public void periodic() {
        odometry.update(imu.getYaw(), getModsPos());
        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("Pitch", Math.abs(MathUtil.inputModulus(imu.getPitch().getDegrees(), -180, 180)));
        SmartDashboard.putNumber("Yaw", Math.abs(MathUtil.inputModulus(imu.getYaw().getDegrees(), -180, 180)));
        SmartDashboard.putNumber("Roll", Math.abs(MathUtil.inputModulus(imu.getRoll().getDegrees(), -180, 180)));

        SmartDashboard.putString("Chassie Speed Calcualted", getChassisSpeed().toString());

        SmartDashboard.putData("Field", field);

        for (MecModule mod : moduels) {
            SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Cancoder", mod.getPos());
            SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Velocity", mod.getVel());
        }
    }

}