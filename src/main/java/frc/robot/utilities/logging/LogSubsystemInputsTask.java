package frc.robot.utilities.logging;

import frc.robot.subsystems.messaging.Messaging;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.GamePieceVision;
import java.util.TimerTask;

import org.littletonrobotics.junction.Logger;

public class LogSubsystemInputsTask extends TimerTask {
	private LogInputs loggingHelper = LogInputs.getInstance();
	private Loggable[] loggingTargets = {
		// TODO: Add things to log here
		Swerve.getInstance(),
		GamePieceVision.getInstance(),
		AprilTagVision.getInstance(),
		Messaging.getInstance()
	};

	@Override
	public void run() {
		for (Loggable target : loggingTargets) {
			loggingHelper.setLoggingTarget(target);
			Logger.getInstance().processInputs(target.getTableName(), loggingHelper);
		}
	}
}
