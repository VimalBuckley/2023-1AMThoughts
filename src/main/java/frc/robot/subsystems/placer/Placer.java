package frc.robot.subsystems.placer;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.EncodedMotorController;
import frc.robot.hardware.SparkMaxMotorController;
import frc.robot.hardware.TalonSRXMotorController;

public class Placer extends SubsystemBase {
    private static Placer instance;
    public static synchronized Placer getInstance() {
        if (instance == null) instance = new Placer();
        return instance;
    }

    private EncodedMotorController armExtensionMotor;
    private EncodedMotorController armAngleMotor;
    private EncodedMotorController intakeAngleMotor;
    private EncodedMotorController intakeRunMotor;
    private GamePiece currentGamePiece;
    private PlacerPosition currentPlacerPosition;

    public Placer() {
        currentGamePiece = GamePiece.Cone;
        currentPlacerPosition = PlacerPosition.AutoZero;
        armExtensionMotor = new TalonSRXMotorController(15);
        armAngleMotor = new SparkMaxMotorController(10, MotorType.kBrushless);
        intakeAngleMotor = new SparkMaxMotorController(12, MotorType.kBrushless);
        intakeRunMotor = new SparkMaxMotorController(13, MotorType.kBrushless);

        armExtensionMotor.setPID(new PIDConstants(0.04, 0, 0))
            .setMinOutput(-0.5)
            .setMaxOutput(0.5);
        
        armAngleMotor.setPID(new PIDConstants(0.4, 0, 0))
            .setAngleTolerance(0.6)
            .setMaxAngle(15.34)
            .setMinOutput(-0.3)
            .setMaxOutput(0.6);

        intakeAngleMotor.setPID(new PIDConstants(1, 0, 0))
            .setMinOutput(-0.3)
            .setMaxOutput(0.3)
            .setBrakeOnIdle(false)
            .setMinAngle(-251);

        intakeRunMotor.setBrakeOnIdle(true);
    }

    public Command setPlacerState(PlacerPosition nextPlacerState, GamePiece nextGamePiece) {
        return Commands.runOnce(
            () -> {
                setPlacerPosition(nextPlacerState);
                setPlacerOutput(nextGamePiece);
            }, this
        ).andThen(Commands.waitUntil(
            this::atTargetState
        ));
    }

    public Command setPlacerState(PlacerPosition nextPlacerState) {
        return setPlacerState(nextPlacerState, currentGamePiece);
    }

    public Command setPlacerState(GamePiece nextGamePiece) {
        return setPlacerState(currentPlacerPosition, nextGamePiece);
    }

    private void setPlacerOutput(GamePiece nextGamePiece) {
        GamePiece current = currentGamePiece;
        GamePiece next = nextGamePiece;
        if (current == next || (current != GamePiece.None && next != GamePiece.None)) {
            intakeRunMotor.setOutput(0);
        } else {
            intakeRunMotor.setOutput(
                next == GamePiece.None ? current.outtakeSpeed : next.intakeSpeed
            );
            currentGamePiece = next;
        }
    }

    private void setPlacerPosition(PlacerPosition nextPlacerPosition) {
        armExtensionMotor.setAngle(nextPlacerPosition.armExtension);
        armAngleMotor.setAngle(nextPlacerPosition.armAngle);
        intakeAngleMotor.setAngle(nextPlacerPosition.intakeAngle);
        currentPlacerPosition = nextPlacerPosition;
    }

    private boolean atTargetState() {
        double armExtension = armExtensionMotor.getAngleRadians();
        double armAngle = armAngleMotor.getAngleRadians();
        double intakeAngle = intakeAngleMotor.getAngleRadians();
        return Math.abs(armExtension - currentPlacerPosition.armExtension )< 1
            && Math.abs(armAngle - currentPlacerPosition.armAngle) < 1
            && Math.abs(intakeAngle - currentPlacerPosition.intakeAngle) < 1; 
    }

    public static enum PlacerPosition {
        AutoZero,
        TeleopZero,
        Bottom,
        Middle,
        Top,
        Substation;

        public double armExtension;
        public double armAngle;
        public double intakeAngle;
    }

    public static enum GamePiece {
        None,
        Cube,
        Cone;

        public double intakeSpeed;
        public double outtakeSpeed;
    }
}