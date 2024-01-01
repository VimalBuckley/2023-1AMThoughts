package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.NavX;
import frc.robot.subsystems.swerve.SwerveConstants.DriveMode;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.GamePieceVision;

public class Swerve extends SubsystemBase implements LoggableInputs {
    private static Swerve instance;
    public static synchronized Swerve getInstance() {
        if (instance == null) instance = new Swerve();
        return instance;
    }

    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    private NavX gyro;
	private AprilTagVision tagVision;
	private GamePieceVision pieceVision;
    private PIDController anglePID;
    private Rotation2d targetAngle;
    private DriveMode driveMode;

    private Swerve() {
        anglePID = new PIDController(4, 0, 0);
		anglePID.enableContinuousInput(-Math.PI, Math.PI);
		anglePID.setTolerance(Math.PI / 32, Math.PI / 32);
		anglePID.setSetpoint(0);
		modules = new SwerveModule[] {
			new SwerveModule(
				SwerveConstants.FRONT_LEFT_DRIVE_MOTOR,
				SwerveConstants.FRONT_LEFT_ANGLE_MOTOR,
				SwerveConstants.FRONT_LEFT_MODULE_TRANSLATION,
				SwerveConstants.MODULE_CONFIG
			),
			new SwerveModule(
				SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR,
				SwerveConstants.FRONT_RIGHT_ANGLE_MOTOR,
				SwerveConstants.FRONT_RIGHT_MODULE_TRANSLATION,
				SwerveConstants.MODULE_CONFIG
			),
			new SwerveModule(
				SwerveConstants.BACK_LEFT_DRIVE_MOTOR,
				SwerveConstants.BACK_LEFT_ANGLE_MOTOR,
				SwerveConstants.BACK_LEFT_MODULE_TRANSLATION,
				SwerveConstants.MODULE_CONFIG
			),
			new SwerveModule(
				SwerveConstants.BACK_RIGHT_DRIVE_MOTOR,
				SwerveConstants.BACK_RIGHT_ANGLE_MOTOR,
				SwerveConstants.BACK_RIGHT_MODULE_TRANSLATION,
				SwerveConstants.MODULE_CONFIG
			),
		};
		gyro = new NavX(I2C.Port.kMXP);
		tagVision = AprilTagVision.getInstance();
		pieceVision = GamePieceVision.getInstance();
		kinematics = new SwerveDriveKinematics(getModuleTranslations());
		odometry = new SwerveDriveOdometry(
			kinematics,
			gyro.getUnwrappedAngle(),
			getModulePositions(),
			tagVision.getRobotPose(new Pose2d())
		);
        driveMode = DriveMode.AngleCentric;
        Shuffleboard.getTab("Display").addBoolean(
			"Gyro Connected", 
			() -> gyro.getAHRS().isConnected()
		);
    }

    public Command followControllerCommand(SwerveController controller) {
        return Commands.run(
            () -> {
				ChassisSpeeds speeds = SwerveConstants.CONTROLLLER_SENS.generateSpeeds(controller);
				targetAngle = controller.getTargetAngle(SwerveConstants.CONTROLLLER_SENS, targetAngle);
                switch (driveMode) {
                    case RobotCentric:
                        driveRobotCentric(
                            speeds
                        );
				        break;
                    case AngleCentric:
                        driveAngleCentric(
                            speeds.vxMetersPerSecond,
							speeds.vyMetersPerSecond,
                            targetAngle
                        );
                        break;
                    case AlignToTarget:
                        driveAlignToTarget(
                            speeds.vxMetersPerSecond,
							speeds.vyMetersPerSecond,
                            targetAngle
                        );
                }
            }, this
        ).beforeStarting(
            () -> targetAngle = getRobotAngle()
        );
    }

    public Command moveToTagCommand(Pose2d relativeTargetPose) {
        return Commands.run(
			() -> {
				Pose2d poseDif = tagVision.getRelativeTagPose(relativeTargetPose).relativeTo(relativeTargetPose);
				driveRobotCentric(
                    new ChassisSpeeds(
						poseDif.getX() * 2,
						poseDif.getY() * 2,
						-poseDif.getRotation().getRadians() * 5
					)
				);
			}, this
		);
    }

    public Command resetGyroCommand() {
        return Commands.runOnce(() -> resetGyro());
    }

    public Command toggleRobotCentricCommand() {
        return Commands.runOnce(
            () -> {
                if (driveMode != DriveMode.RobotCentric) {
					driveMode = DriveMode.RobotCentric;
				} else {
					driveMode = DriveMode.AngleCentric;
					targetAngle = getRobotAngle();
				}
            }  
        );
    }

	public Command toggleAlignToTargetCommand() {
		return Commands.startEnd(
			() -> {
				driveMode = DriveMode.AlignToTarget;
				targetAngle = getRobotAngle();
			},
			() -> {
				driveMode = DriveMode.AngleCentric;
				targetAngle = getRobotAngle();
			}
		);
	}

    public void driveRobotCentric(ChassisSpeeds targetVelocities) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
			discretize(targetVelocities)
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			states,
			SwerveConstants.MAX_LINEAR_SPEED_MPS
		);
		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(states[i]);
		}
    }

    public void driveAngleCentric(
        double forwardVelocity,
        double sidewaysVelocity,
        Rotation2d targetAngle    
    ) {
        driveRobotCentric(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardVelocity,
                sidewaysVelocity,
                calculateRotationalVelocityToTarget(targetAngle),
                getRobotAngle()
            )
		);
    }

    public void driveAlignToTarget(
		double forwardVelocity,
		double leftVelocity,
		Rotation2d aligningAngle
	) {
		driveRobotCentric(
			new ChassisSpeeds(
				ChassisSpeeds.fromFieldRelativeSpeeds(
					forwardVelocity,
					leftVelocity, 
					0, 
					getRobotAngle()
				).vxMetersPerSecond, 
				pieceVision.getHorizontalOffset(new Rotation2d()).getDegrees() / 10,
				calculateRotationalVelocityToTarget(aligningAngle)
			)
		);
	}

    @Override
    public void periodic() {
        Rotation2d gyroAngle = gyro.getUnwrappedAngle();
		SwerveModulePosition[] modulePositions = getModulePositions();
		odometry.update(gyroAngle, modulePositions);
		if (tagVision.seesTag()) resetPose(tagVision.getRobotPose(getRobotPose()));
    }

	@Override
	public void toLog(LogTable table) {
		table.put(
			"Front Left Module Velocity (M/S)",
			modules[0].getState().speedMetersPerSecond
		);
		table.put(
			"Front Left Module Angle (Radians)",
			modules[0].getState().angle.getRadians()
		);
		table.put(
			"Front Right Module Velocity (M/S)",
			modules[1].getState().speedMetersPerSecond
		);
		table.put(
			"Front Right Module Angle (Radians)",
			modules[1].getState().angle.getRadians()
		);
		table.put(
			"Back Left Module Velocity (M/S)",
			modules[2].getState().speedMetersPerSecond
		);
		table.put(
			"Back Left Module Angle (Radians)",
			modules[2].getState().angle.getRadians()
		);
		table.put(
			"Back Right Module Velocity (M/S)",
			modules[3].getState().speedMetersPerSecond
		);
		table.put(
			"Back Right Module Angle (Radians)",
			modules[3].getState().angle.getRadians()
		);
		Logger.getInstance().recordOutput(
			"Swerve Odometry", 
			getRobotPose()
		);
		Logger.getInstance().recordOutput(
			"Module States", 
			getModuleStates()
		);
	}

	@Override
	public void fromLog(LogTable table) {}

    public Pose2d getRobotPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getRobotAngle() {
        return gyro.getOffsetedAngle();
    }

    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(gyro.getUnwrappedAngle(), getModulePositions(), newPose);
    }

    public void resetGyro() {
        gyro.zeroGyro();
    }

    public void offsetGyro(Rotation2d offset) {
        gyro.zeroGyroWithOffset(offset);
    }

    private double calculateRotationalVelocityToTarget(Rotation2d targetRotation) {
		double rotationalVelocity = anglePID.calculate(
			getRobotAngle().getRadians(), 
			targetRotation.getRadians()
		);
		if (anglePID.atSetpoint()) {
			rotationalVelocity = 0;
		}
		return rotationalVelocity;
	}

	/**
	 * Fixes situation where robot drifts in the direction it's rotating in if
	 * turning and translating at the same time
	 * 
	 * @see <a href="https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964">Chief Delphi</a>
	 */
	private ChassisSpeeds discretize(ChassisSpeeds originalChassisSpeeds) {
		double vx = originalChassisSpeeds.vxMetersPerSecond;
		double vy = originalChassisSpeeds.vyMetersPerSecond;
		double omega = originalChassisSpeeds.omegaRadiansPerSecond;
		double dt = 0.02; // This should be the time these values will be used, so normally just the loop time
		Pose2d desiredDeltaPose = new Pose2d(
            vx * dt,
            vy * dt,
            new Rotation2d(omega * dt)
        );
		Twist2d twist = new Pose2d().log(desiredDeltaPose);
		return new ChassisSpeeds(
            twist.dx / dt,
            twist.dy / dt,
            twist.dtheta / dt
        );
	}

    private Translation2d[] getModuleTranslations() {
		Translation2d[] translations = new Translation2d[modules.length];
		for (int i = 0; i < modules.length; i++) {
			translations[i] = modules[i].getTranslationFromCenter();
		}
		return translations;
	}

	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			positions[i] = modules[i].getPosition();
		}
		return positions;
	}
}