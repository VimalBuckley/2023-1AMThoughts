package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.messaging.Messaging;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
	private CommandXboxController xbox;
	private Swerve swerve;
	private Messaging messaging;
	private Command autoCommand;
	private SendableChooser<Command> autonChooser;

	private final int DRIVER_PORT = 2;

	public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
		swerve = Swerve.getInstance();
		messaging = Messaging.getInstance();
		setupAuto();
		setupDriveController();
	}

	public void setupAuto() {
		autonChooser = new SendableChooser<Command>();
		autonChooser.setDefaultOption("No Auto", null);
		Shuffleboard.getTab("Display").add(
			"Auto Route", 
			autonChooser
		);
	}

	public void setupDriveController() {
		xbox = new CommandXboxController(DRIVER_PORT);
		swerve.setDefaultCommand(swerve.followControllerCommand(xbox));
		
		Trigger switchDriveModeButton = xbox.x();
		Trigger resetGyroButton = xbox.a();
		Trigger alignToTargetButton = xbox.rightBumper();
		Trigger cancelationButton = xbox.start();
		Trigger moveToAprilTagButton = xbox.leftBumper();

        moveToAprilTagButton.whileTrue(swerve.moveToTagCommand(new Pose2d(1, 0, new Rotation2d())));
        switchDriveModeButton.onTrue(swerve.toggleRobotCentricCommand());
		resetGyroButton.onTrue(swerve.resetGyroCommand());
		alignToTargetButton.whileTrue(swerve.toggleAlignToTargetCommand());
		cancelationButton.onTrue(Commands.runOnce(
			() -> CommandScheduler.getInstance().cancelAll())
		);
	}

	public Command rumbleCommand(double timeSeconds) {
		return Commands.startEnd(
			() -> xbox.getHID().setRumble(RumbleType.kBothRumble, 0.5),
			() -> xbox.getHID().setRumble(RumbleType.kBothRumble, 0)
		).withTimeout(timeSeconds);
	}

	public void autonomousInit() {
		messaging.enableMessaging(true);
		messaging.addMessage("Auto Started");
		autoCommand = autonChooser.getSelected();
		if (autoCommand != null) {
			autoCommand.schedule();
		} else {
			messaging.addMessage("No Auto Command Selected");
		}
	}

	public void teleopInit() {
		messaging.enableMessaging(true);
		messaging.addMessage("Teleop Started");
		if (autoCommand != null) {
			autoCommand.cancel();
		}
	}
}