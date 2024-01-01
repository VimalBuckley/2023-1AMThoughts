package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public interface SwerveController {
    public double getForwardInput();
    public double getSidewaysInput();
    public double getRotationalInput();
    public Rotation2d getTargetAngle(SwerveSens sens, Rotation2d currentTarget);
    public double getVelocityScalar();

    public static SwerveController fromXbox(CommandXboxController xbox){
        return new SwerveController() {
            @Override
            public double getForwardInput() {
                return -xbox.getLeftY();
            }

            @Override
            public double getSidewaysInput() {
                return -xbox.getLeftX();
            }

            @Override
            public double getRotationalInput() {
                return -xbox.getRightX();
            }

            @Override
            public Rotation2d getTargetAngle(SwerveSens sens, Rotation2d currentTarget) {
                if (Math.abs(xbox.getRightY()) > 0.5) {
                    return Rotation2d.fromDegrees(90 - 90 * Math.signum(-xbox.getRightY()));
                }
                return Rotation2d.fromDegrees(currentTarget.getDegrees() -xbox.getRightX() * sens.rotational());
            }

            @Override
            public double getVelocityScalar() {
                return xbox.getLeftTriggerAxis();
            }
            
        };
    }
}
