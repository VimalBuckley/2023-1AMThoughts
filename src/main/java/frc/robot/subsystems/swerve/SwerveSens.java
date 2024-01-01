package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** A record that represents the sensitivities of our teleop driving. All should be positive
 * @param forward The max forward sensitivity
 * @param sideways The max sideways sensitivity
 * @param rotational The max rotational sensitivity
 * @param min The minimum sensitivity. When we scale the sensitivites
 * for slow mode, if any fall below this value, this value will be used instead
 */
public record SwerveSens(
    double forward, 
    double sideways, 
    double rotational,
    double min
) {
    /** A method to generate a chassis speed based on a controller. Handles scaling for slow mode as well */
    public ChassisSpeeds generateSpeeds(SwerveController controller) {
        double scalar = controller.getVelocityScalar();
        SwerveSens scaledSens = new SwerveSens(
            Math.max(Math.abs(forward * scalar), min), 
            Math.max(Math.abs(sideways * scalar), min),
            Math.max(Math.abs(rotational * scalar), min), 
            min
        );
        return new ChassisSpeeds(
            scaledSens.forward * controller.getForwardInput(),
            scaledSens.sideways * controller.getSidewaysInput(),
            scaledSens.rotational * controller.getRotationalInput()
        );
    }
}
