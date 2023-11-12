package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Rotation2d;

import frc.lib.math.Conversions;
// import frc.lib.math.CTREModuleState;
// import frc.lib.math.Conversions;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// //import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
public class MecModule {

    private TalonFX driveMotor;
    private TalonFXSimCollection driveMotorSim;
    public int moduleNumber;

    public MecModule(int moduleNumber) {
        this.moduleNumber = moduleNumber;

        driveMotor = new WPI_TalonFX(moduleNumber);
        driveMotorSim = driveMotor.getSimCollection();

    }

    /** curetnt percent output. [-5,5] */
    public void driveMoter(double Velocity) {
        SmartDashboard.putNumber("Mod" + moduleNumber + " Set Velocity", Velocity);

        driveMotor.set(ControlMode.PercentOutput, Velocity / 5);
        driveMotorSim.setSupplyCurrent(Velocity / 5 * 12);

        // Conversions.MPSToFalcon(kWheelRadiusInches, 8, 10.14)
        // m_driveSim.update(0.02);

        // m_leftDriveSim.setQuadratureRawPosition(distanceToNativeUnits(m_driveSim.getLeftPositionMeters()));
    }

    public double getPos() {
        return Conversions.falconToDegrees(driveMotor.getSelectedSensorPosition(),
                10.41);
    }

    public double getVel() {
        return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
                8, 10.41);
    }

    @Override
    public String toString() {
        return (moduleNumber + " Velocity: " + getVel());
    }
}
// public void setDesiredState(MecanumDriveWheelSpeeds desiredState, boolean
// isOpenLoop) {
// if (isOpenLoop) {
// double percentOutput = desiredState.speedMetersPerSecond * .85;
// driveMotor.set(ControlMode.PercentOutput, percentOutput);
// } else {
// // double referenceVelocity = desiredState.speedMetersPerSecond;
// // double arbFeedForward =
// // feedforward.calculate(desiredState.speedMetersPerSecond) /
// // // kSwerve.nominalVoltage;
// // driveMotor.set(
// // TalonFXControlMode.Velocity,
// // desiredState.speedMetersPerSecond
// // / (Math.PI * kSwerve.wheelDiameter * kSwerve.driveGearRatio / 2048 * 10));
// // driveMotor.feed();
// }
// }