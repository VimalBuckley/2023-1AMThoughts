package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An interface representing a controller which can control swerve during teleop */
public interface SwerveController {
    /** This should return a number between 0 and 1 representing how
     * fast foward we want to go. Forward is positive */
    public double getForwardInput();
    /** This should return a number between 0 and 1 representing how
     * fast sideways we want to go. Left is positive */
    public double getSidewaysInput();
    /** This should return a number between 0 and 1 representing how
     * fast we want to turn. Counter-clockwise is positive */
    public double getRotationalInput();
    /**
     * A method to get the next target angle
     * @param sens The sensitivites being worked with
     * @param currentTarget The current target angle
     * @return the next target angle
     */
    public Rotation2d getTargetAngle(SwerveSens sens, Rotation2d currentTarget);
    /** This should return a number between 0 and 1 to scale all velocities by.
     * In effect, this is our slow mode. */
    public double getVelocityScalar();

    /** A factory that converts an XboxController into a SwerveController */
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
