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
	private SwerveModuleConfig config;

	/**
	 * Creates a new swerve module with the given attributes
	 * @param driveMotor The motor to be used for driving. Should already be configured
	 * @param angleMotor The motor to be used for turning. Should already be configured
	 * @param translationToCenter The translation from the center of rotation of the module
	 * to the center of rotation of the robot, which is normally the center of the robot
	 * @param config Misc info about gear ratios and wheel diameter. Probably the same for all modules
	 */
	public SwerveModule(
		EncodedMotorController driveMotor,
		EncodedMotorController angleMotor,
		Translation2d translationToCenter,
		SwerveModuleConfig config
	) {
		this.driveMotor = driveMotor;
		this.angleMotor = angleMotor;
		this.translationFromCenter = translationToCenter;
		this.config = config;
	}

	public void drive(SwerveModuleState initialTargetState) {
		SwerveModuleState targetState = ExtendedMath.optimizeModuleState(
			initialTargetState, 
			getState().angle,
			angleMotor.hasContinuousRotation()
		);
		setVelocity(
			targetState.speedMetersPerSecond * 
            // Scale velocity by how far wheel is from target
			Math.abs(targetState.angle.minus(getState().angle).getCos())
		);
		setAngle(targetState.angle);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			driveMotor.getAngularVelocity()
				.times(config.driveRatio)
				.times(config.diameterMeters)
				.div(2)
				.getRadians(),
			angleMotor.getAngle().times(config.angleRatio)
		);
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			driveMotor.getAngle()
				.div(2 * Math.PI)
				.times(config.driveRatio)
				.times(config.diameterMeters)
				.times(Math.PI)
				.getRadians(),
			getState().angle
		);
	}

	public Translation2d getTranslationFromCenter() {
		return translationFromCenter;
	}

	public void setAngle(Rotation2d angle) {
		angleMotor.setAngle(angle.div(config.angleRatio));
	}

	public void setVelocity(double targetVelocityMetersPerSecond) {
		driveMotor.setAngularVelocity(
			Rotation2d.fromRadians(
				targetVelocityMetersPerSecond * 2 /
				(config.driveRatio * config.angleRatio)
			)
		);
	}

	public static record SwerveModuleConfig(
        double driveRatio, 
        double angleRatio, 
        double diameterMeters
    ) {}
}