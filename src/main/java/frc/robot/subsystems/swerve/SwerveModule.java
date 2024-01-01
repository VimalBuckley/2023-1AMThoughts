package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.EncodedMotorController;
import frc.robot.utilities.ExtendedMath;

public class SwerveModule {
	private EncodedMotorController driveMotor;
	private EncodedMotorController angleMotor;
	private Translation2d translationFromCenter;

	public SwerveModule(
		EncodedMotorController driveMotor,
		EncodedMotorController angleMotor,
		Translation2d translationToCenter
	) {
		this.driveMotor = driveMotor;
		this.angleMotor = angleMotor;
		this.translationFromCenter = translationToCenter;
	}

	public void drive(SwerveModuleState initialTargetState) {
		SwerveModuleState targetState = ExtendedMath.optimizeModuleState(
			initialTargetState, 
			getModuleState().angle,
			angleMotor.hasContinuousRotation()
		);
		setModuleVelocity(
			targetState.speedMetersPerSecond * 
            // Scale velocity by how far wheel is from target
			Math.abs(targetState.angle.minus(getModuleState().angle).getCos())
		);
		setModuleAngle(targetState.angle);
	}

	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(
			driveMotor.getAngularVelocity()
				.times(SwerveConstants.DRIVE_RATIO)
				.times(SwerveConstants.WHEEL_DIAMETER_METERS)
				.div(2)
				.getRadians(),
			angleMotor.getAngle().times(SwerveConstants.ANGLE_RATIO)
		);
	}

	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(
			driveMotor.getAngle()
				.div(2 * Math.PI)
				.times(SwerveConstants.DRIVE_RATIO)
				.times(SwerveConstants.WHEEL_DIAMETER_METERS)
				.times(Math.PI)
				.getRadians(),
			getModuleState().angle
		);
	}

	public Translation2d getTranslationFromCenter() {
		return translationFromCenter;
	}

	public void setModuleAngle(Rotation2d angle) {
		angleMotor.setAngle(angle.div(SwerveConstants.ANGLE_RATIO));
	}

	public void setModuleVelocity(double targetVelocityMetersPerSecond) {
		driveMotor.setAngularVelocity(
			Rotation2d.fromRadians(
				targetVelocityMetersPerSecond * 2 /
				(SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER_METERS)
			)
		);
	}
}