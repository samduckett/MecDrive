package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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

    public int moduleNumber;

    public MecModule(int moduleNumber) {
        moduleNumber = this.moduleNumber;

        driveMotor = new WPI_TalonFX(1);
    }

    /** curetnt percent output. [-5,5] */
    public void driveMoter(double Velocity) {
        driveMotor.set(ControlMode.PercentOutput, Velocity / 5);
        ;
    }

    public double getPosition() {
        return Conversions.falconToDegrees(driveMotor.getSelectedSensorPosition(),
                10.41);
    }

    public double getVelocity() {
        return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
                8, 10.41);
    }

    public double getPos() {
        return getPosition();
    }

    public double getVel() {
        return getVelocity();
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