package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.CANConstants;
import frc.robot.hardware.EncodedMotorController;
import frc.robot.hardware.TalonFXMotorController;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleConfig;

public class SwerveConstants {
    // TODO: Change these!
    public static final double MAX_LINEAR_SPEED_MPS = 5.088;

    public static final SwerveModuleConfig MODULE_CONFIG = new SwerveModuleConfig(
        1 / 5.0, 
        1 / 6.75,
        0.0762
    );

    public static enum DriveMode {
        AngleCentric,
        RobotCentric,
        AlignToTarget
    }

    public static SwerveSens CONTROLLLER_SENS = new SwerveSens(
        4,  
        4, 
        3.5,
        0.2 
    );

    public static final EncodedMotorController FRONT_LEFT_DRIVE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_FRONT_LEFT_DRIVE_ID)
            .configInversion(true)
            .configCurrentLimit(35)
            .configPID(new PIDConstants(0.075, 0, 0)
        );
    public static final EncodedMotorController FRONT_LEFT_ANGLE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_FRONT_LEFT_ANGLE_ID)
            .configInversion(false)
            .configCurrentLimit(25)
            .configPID(new PIDConstants(0.3, 0, 0)
        );
    public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(
        0.3175,
        0.2413
    );

    public static final EncodedMotorController FRONT_RIGHT_DRIVE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_FRONT_RIGHT_DRIVE_ID)
            .configInversion(false)
            .configCurrentLimit(35)
            .configPID(new PIDConstants(0.05, 0, 0)
        );
    public static final EncodedMotorController FRONT_RIGHT_ANGLE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_FRONT_RIGHT_ANGLE_ID)
            .configInversion(false)
            .configCurrentLimit(25)
            .configPID(new PIDConstants(0.3, 0, 0)
        );
    public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(
        0.3175,
        -0.2413
    );

    public static final EncodedMotorController BACK_LEFT_DRIVE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_BACK_LEFT_DRIVE_ID)
            .configInversion(true)
            .configCurrentLimit(35)
            .configPID(new PIDConstants(0.075, 0, 0)
        );
    public static final EncodedMotorController BACK_LEFT_ANGLE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_BACK_LEFT_ANGLE_ID)
            .configInversion(false)
            .configCurrentLimit(25)
            .configPID(new PIDConstants(0.25, 0, 0)
        );
    public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(
        -0.3175,
        0.2431
    );

    public static final EncodedMotorController BACK_RIGHT_DRIVE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_BACK_RIGHT_DRIVE_ID)
            .configInversion(false)
            .configCurrentLimit(35)
            .configPID(new PIDConstants(0.05, 0, 0)
        );
    public static final EncodedMotorController BACK_RIGHT_ANGLE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_BACK_RIGHT_ANGLE_ID)
            .configInversion(false)
            .configCurrentLimit(25)
            .configPID(new PIDConstants(0.3, 0, 0)
        );
    public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(
        -0.3175,
        -0.2413
    );
}