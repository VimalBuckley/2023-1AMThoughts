package frc.robot.subsystems.placer;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.EncodedMotorController;
import frc.robot.hardware.SparkMaxMotorController;
import frc.robot.hardware.TalonSRXMotorController;
import static frc.robot.subsystems.placer.PlacerConstants.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Placer extends SubsystemBase implements LoggableInputs {
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

    public Command runPlacerCommand(PlacerOutput output) {
        return Commands.startEnd(
            () -> setPlacerOutput(output),
            () -> setPlacerOutput(PlacerOutput.Off)
        );
    }

    public Command runPlacerAndZeroCommand(PlacerOutput output) {
        return Commands.startEnd(
            () -> setPlacerOutput(output),
            () -> {
                setPlacerOutput(PlacerOutput.Off);
                setPlacerPosition(PlacerPosition.TeleopZero);
            },
            this  
        );
    }

    public Command movePlacerCommand(PlacerPosition position) {
        return Commands.runOnce(
            () -> setPlacerPosition(position), 
            this
        ).andThen(Commands.waitUntil(this::atTargetState));
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Target Game Piece", currentGamePiece.name());
        table.put("Target Placer Position", currentPlacerPosition.name());
        table.put("Current Arm Extension", armExtensionMotor.getAngle().getRadians());
        table.put("Current Arm Angle", armAngleMotor.getAngle().getRadians());
        table.put("Current Intake Intake", intakeAngleMotor.getAngle().getRadians());
    }

    @Override
    public void fromLog(LogTable table) {}

    private void setPlacerOutput(PlacerOutput output) {
        switch (output) {
            default:
            case Off:
                intakeRunMotor.setOutput(0);
                break;
            case PickupCone:
                intakeRunMotor.setOutput(GamePiece.Cone.intakeOutput);
                currentGamePiece = GamePiece.Cone;
                break;
            case PickupCube:
                intakeRunMotor.setOutput(GamePiece.Cube.intakeOutput);
                currentGamePiece = GamePiece.Cube;
                break;
            case Place:
                intakeRunMotor.setOutput(currentGamePiece.placeOutput);
                currentGamePiece = GamePiece.None;
                break;
        }
    }

    private void setPlacerPosition(PlacerPosition nextPlacerPosition) {
        armExtensionMotor.setAngle(nextPlacerPosition.armExtension);
        armAngleMotor.setAngle(nextPlacerPosition.armAngle);
        intakeAngleMotor.setAngle(nextPlacerPosition.intakeAngle);
        currentPlacerPosition = nextPlacerPosition;
    }

    private boolean atTargetState() {
        Rotation2d armExtension = armExtensionMotor.getAngle();
        Rotation2d armAngle = armAngleMotor.getAngle();
        Rotation2d intakeAngle = intakeAngleMotor.getAngle();
        return Math.abs(armExtension.minus(currentPlacerPosition.armExtension).getRadians())< 1
            && Math.abs(armAngle.minus(currentPlacerPosition.armAngle).getRadians()) < 1
            && Math.abs(intakeAngle.minus(currentPlacerPosition.intakeAngle).getRadians()) < 1; 
    }

    public static enum PlacerPosition {
        AutoZero(AUTO_ZERO_ARM_EXTENSION, AUTO_ZERO_ARM_ANGLE, AUTO_ZERO_INTAKE_ANGLE),
        TeleopZero(TELEOP_ZERO_ARM_EXTENSION, TELEOP_ZERO_ARM_ANGLE, TELEOP_ZERO_INTAKE_ANGLE),
        Bottom(BOTTOM_ARM_EXTENSION, BOTTOM_ARM_ANGLE, BOTTOM_INTAKE_ANGLE),
        Middle(MIDDLE_ARM_EXTENSION, MIDDLE_ARM_ANGLE, MIDDLE_INTAKE_ANGLE),
        Top(TOP_ARM_EXTENSION, TOP_ARM_ANGLE, TOP_INTAKE_ANGLE),
        Substation(SUBSTATION_ARM_EXTENSION, SUBSATION_ARM_ANGLE, SUBSTATION_INTAKE_ANGLE);

        public Rotation2d armExtension;
        public Rotation2d armAngle;
        public Rotation2d intakeAngle;

        private PlacerPosition(Rotation2d armExtension, Rotation2d armAngle, Rotation2d intakeAngle) {
            this.armExtension = armExtension;
            this.armAngle = armAngle;
            this.intakeAngle = intakeAngle;
        }
    }

    public static enum PlacerOutput {
        Place,
        PickupCone,
        PickupCube,
        Off
    }

    public static enum GamePiece {
        None(0, 0),
        Cube(CUBE_INTAKE_OUTPUT, CUBE_PLACE_OUTPUT),
        Cone(CONE_INTAKE_OUTPUT, CONE_PLACE_OUTPUT);

        public double intakeOutput;
        public double placeOutput;

        private GamePiece(double intakeOutput, double placeOutput) {
            this.intakeOutput = intakeOutput;
            this.placeOutput = placeOutput;
        }
    }
}