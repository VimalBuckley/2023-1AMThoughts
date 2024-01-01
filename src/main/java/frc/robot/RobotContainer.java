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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.messaging.Messaging;
import frc.robot.subsystems.placer.Placer;
import frc.robot.subsystems.placer.Placer.PlacerOutput;
import frc.robot.subsystems.placer.Placer.PlacerPosition;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveController;

public class RobotContainer {
	private CommandXboxController xbox;
	private CommandJoystick flightSim;
	private Swerve swerve;
	private Placer placer;
	private Messaging messaging;
	private Command autoCommand;
	private SendableChooser<Command> autonChooser;

	private final int DRIVER_PORT = 2;
	private final int OPERATOR_PORT = 1;

	public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
		swerve = Swerve.getInstance();
		placer = Placer.getInstance();
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
		swerve.setDefaultCommand(swerve.followControllerCommand(
			SwerveController.fromXbox(xbox)
		));
		
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

	public void setupOperatorController() {
		flightSim = new CommandJoystick(OPERATOR_PORT);

		Trigger placeButton = flightSim.button(1);
		Trigger intakeConeButton = flightSim.button(6);	
		Trigger intakeCubeButton = flightSim.button(12);
		Trigger goBottomButton = flightSim.button(9);
		Trigger goMiddleButton = flightSim.button(7);
		Trigger goTopButton = flightSim.button(8);
		Trigger goSubstationButton = flightSim.button(10);
		Trigger goZeroButton = flightSim.button(2);

		placeButton.whileTrue(placer.runPlacerAndZeroCommand(PlacerOutput.Place));
		intakeConeButton.whileTrue(placer.runPlacerAndZeroCommand(PlacerOutput.PickupCone));
		intakeCubeButton.whileTrue(placer.runPlacerAndZeroCommand(PlacerOutput.PickupCube));
		goBottomButton.onTrue(placer.movePlacerCommand(PlacerPosition.Bottom));
		goMiddleButton.onTrue(placer.movePlacerCommand(PlacerPosition.Middle));
		goTopButton.onTrue(placer.movePlacerCommand(PlacerPosition.Top));
		goSubstationButton.onTrue(placer.movePlacerCommand(PlacerPosition.Substation));
		goZeroButton.onTrue(placer.movePlacerCommand(PlacerPosition.TeleopZero));
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