package frc.robot.hardware;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.util.Units;

public class SparkMaxMotorController extends CANSparkMax implements EncodedMotorController {
	private double angleTolerance;

	public SparkMaxMotorController(int deviceID, MotorType type) {
		super(deviceID, type);
		angleTolerance = 0;
	}

    @Override
	public double getAngleRadians() {
		return Units.rotationsToRadians(getEncoder().getPosition());
	}

    @Override
	public void setAngle(double position) {
		if (Math.abs(position - getAngleRadians())  < angleTolerance) {
			position = getAngleRadians();
		} 
		getPIDController()
			.setReference(Units.radiansToRotations(position), ControlType.kPosition);
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
	public double getAngularVelocity() {
		return Units.rotationsPerMinuteToRadiansPerSecond(getEncoder().getVelocity());
	}
	
    @Override
	public void setAngularVelocity(double velocity) {
		getPIDController()
			.setReference(
				Units.radiansPerSecondToRotationsPerMinute(velocity),
				ControlType.kVelocity
			);
	}

	@Override
	public boolean hasContinuousRotation() {
		return true;
	}

	@Override
	public EncodedMotorController setCurrentLimit(int currentLimit) {
		setSmartCurrentLimit(currentLimit);
		return this;
	}

	@Override
	public EncodedMotorController setPID(PIDConstants pid) {
		SparkMaxPIDController controller = getPIDController();
		controller.setP(pid.kP);
		controller.setI(pid.kI);
		controller.setD(pid.kD);
		return this;
	}

	@Override
	public EncodedMotorController setMinAngle(double minPosition) {
		setSoftLimit(SoftLimitDirection.kReverse, (float) Units.radiansToRotations(minPosition));
		return this;
	}

	@Override
	public EncodedMotorController setMaxAngle(double maxPosition) {
		setSoftLimit(SoftLimitDirection.kForward, (float) Units.radiansToRotations(maxPosition));
		return this;
	}

	@Override
	public EncodedMotorController setMinOutput(double minOutput) {
		SparkMaxPIDController controller = getPIDController();
		controller.setOutputRange(minOutput, controller.getOutputMax());
		return this;
	}

	@Override
	public EncodedMotorController setMaxOutput(double maxOutput) {
		SparkMaxPIDController controller = getPIDController();
		controller.setOutputRange(controller.getOutputMin(), maxOutput);
		return this;
	}

	@Override
	public EncodedMotorController setInversion(boolean shouldInvert) {
		setInverted(shouldInvert);
		return this;
	}

	@Override
	public EncodedMotorController setBrakeOnIdle(boolean shouldBreak) {
        setIdleMode(
          shouldBreak
          ? IdleMode.kBrake
          : IdleMode.kCoast  
        );
		return this;
	}

	@Override
	public EncodedMotorController setAngleTolerance(double tolerance) {
		angleTolerance = tolerance;
		return this;
	}
}