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
    public static Placer instance;
    public static synchronized Placer getInstance() {
        if (instance == null) instance = new Placer();
        return instance;
    }

    private EncodedMotorController armExtensionMotor;
    private EncodedMotorController armAngleMotor;
    private EncodedMotorController intakeAngleMotor;
    private EncodedMotorController intakeRunMotor;

    public Placer() {
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

    public Command setPlacerPosition(PlacerState targetState) {
        return Commands.runOnce(
          () -> {
            armExtensionMotor.setAngle(targetState.armExtension);
            armAngleMotor.setAngle(targetState.armAngle);
            intakeAngleMotor.setAngle(targetState.intakeAngle);
          }, this  
        ).until(
            () -> {
                double armExtension = Math.abs(armExtensionMotor.getAngleRadians());
                double armAngle = Math.abs(armAngleMotor.getAngleRadians());
                double intakeAngle = Math.abs(intakeAngleMotor.getAngleRadians());
                return armExtension < 1
                    && armAngle < 1
                    && intakeAngle < 1;
            }
        );
    }

    public Command setPlacerOutput(PlacerOutput targetOutput) {
        return Commands.runOnce(
            () -> intakeRunMotor.setOutput(targetOutput.intakeSpeed)  
        );
    }

    public static enum PlacerState {
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

    public static enum PlacerOutput {
        Off,
        PickupCone,
        PlaceCone,
        PickupCube,
        PlaceCube;

        double intakeSpeed;
    }
}