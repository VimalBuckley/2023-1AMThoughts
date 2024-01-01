package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record SwerveSens(
    double forward, 
    double sideways, 
    double rotational,
    double min
) {
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
