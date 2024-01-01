package frc.robot.hardware;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;

public class SparkMaxMotorController extends CANSparkMax implements EncodedMotorController {
	private Rotation2d tolerance;

	public SparkMaxMotorController(int deviceID, MotorType type) {
		super(deviceID, type);
		tolerance = new Rotation2d(); // default tolerance to 0
		enableSoftLimit(SoftLimitDirection.kForward, false);
		enableSoftLimit(SoftLimitDirection.kReverse, false);
	}

    @Override
	public Rotation2d getAngle() {
		return Rotation2d.fromRotations(getEncoder().getPosition());
	}

    @Override
	public void setAngle(Rotation2d angle) {
		if (Math.abs(angle.minus(getAngle()).getRadians()) < tolerance.getRadians()) {
			angle = getAngle();
		} 
		getPIDController()
			.setReference(angle.getRotations(), ControlType.kPosition);
	}

    @Override
	public void setOutput(double output) {
		set(output);
	}

    @Override
	public double getPercentOutput() {
		return get();
	}

    @Override
	public Rotation2d getAngularVelocity() {
		return Rotation2d.fromRotations(getEncoder().getVelocity() / 60);
	}
	
    @Override
	public void setAngularVelocity(Rotation2d velocity) {
		getPIDController()
			.setReference(
				velocity.times(60).getRotations(),
				ControlType.kVelocity
			);
	}

	@Override
	public boolean hasContinuousRotation() {
		return true;
	}

	@Override
	public SparkMaxMotorController configCurrentLimit(int currentLimit) {
		setSmartCurrentLimit(currentLimit);
		return this;
	}

	@Override
	public SparkMaxMotorController configPID(PIDConstants pid) {
		SparkMaxPIDController controller = getPIDController();
		controller.setP(pid.kP);
		controller.setI(pid.kI);
		controller.setD(pid.kD);
		return this;
	}

	@Override
	public SparkMaxMotorController configMinAngle(Rotation2d min) {
		enableSoftLimit(SoftLimitDirection.kReverse, true);
		setSoftLimit(SoftLimitDirection.kReverse, (float) min.getRotations());
		return this;
	}

	@Override
	public SparkMaxMotorController configMaxAngle(Rotation2d max) {
		enableSoftLimit(SoftLimitDirection.kForward, true);
		setSoftLimit(SoftLimitDirection.kForward, (float) max.getRotations());
		return this;
	}

	@Override
	public SparkMaxMotorController configMinOutput(double minOutput) {
		SparkMaxPIDController controller = getPIDController();
		controller.setOutputRange(minOutput, controller.getOutputMax());
		return this;
	}

	@Override
	public SparkMaxMotorController configMaxOutput(double maxOutput) {
		SparkMaxPIDController controller = getPIDController();
		controller.setOutputRange(controller.getOutputMin(), maxOutput);
		return this;
	}

	@Override
	public SparkMaxMotorController configInversion(boolean shouldInvert) {
		setInverted(shouldInvert);
		return this;
	}

	@Override
	public SparkMaxMotorController configBrakeOnIdle(boolean shouldBreak) {
        setIdleMode(
          shouldBreak
          ? IdleMode.kBrake
          : IdleMode.kCoast  
        );
		return this;
	}

	@Override
	public SparkMaxMotorController configAngleTolerance(Rotation2d tolerance) {
		this.tolerance = tolerance;
		return this;
	}
}