package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Limelight;

public class GamePieceVision extends SubsystemBase implements LoggableInputs {
    private static GamePieceVision instance;
    public static synchronized GamePieceVision getInstance() {
        if (instance == null) instance = new GamePieceVision();
        return instance;
    }

    // TODO: Change these!
    private final double LIMELIGHT_HEIGHT_METERS = 0.232;
	private final double GAMEPIECE_HALF_HEIGHT_METERS = 0.16;
	private final Rotation2d LIMELIGHT_ANGLE = Rotation2d.fromDegrees(-12);
    private Limelight limelight;

    public GamePieceVision() {
        limelight = new Limelight("limelight-haha");
    }

    public boolean seesPiece() {
        return limelight.hasValidTargets();
    }

    /** Returns the translation of the targeted gamepiece relative to the camera */
    public Translation2d getTranslation(Translation2d defaultTranslation) {
        if (!seesPiece()) return defaultTranslation;
		double forwardDistance = 
			(LIMELIGHT_HEIGHT_METERS - GAMEPIECE_HALF_HEIGHT_METERS) / 
			Math.tan(
				LIMELIGHT_ANGLE.plus(
					getVerticalOffset(new Rotation2d())
				).getRadians()
			);
		return new Translation2d(
			forwardDistance,
			forwardDistance * Math.tan(
				getVerticalOffset(new Rotation2d())
				.getRadians()
			)
		);
    }

    /** Returns the horizontal angle from the camera to the gamepiece */
    public Rotation2d getHorizontalOffset(Rotation2d defaultOffset) {
        return limelight.getHorizontalOffsetFromCrosshair().orElse(defaultOffset);
    }

    /** Returns the verical angle from the camera to the gamepiece */
    public Rotation2d getVerticalOffset(Rotation2d defaultOffset) {
        return limelight.getVerticalOffsetFromCrosshair().orElse(defaultOffset);
    }

    /** Returns a percent (0-1) of how much of the screen is taken up by the gamepiece */
    public double getTakenArea(double defaultArea) {
        return limelight.getTargetArea().orElse(defaultArea);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Sees Piece", seesPiece());
        Logger.getInstance().recordOutput(
            "Piece Translation", 
            new Pose2d(getTranslation(new Translation2d()), new Rotation2d())
        );
    }

    @Override
    public void fromLog(LogTable table) {}
}
