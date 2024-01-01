package frc.robot.hardware;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public interface EncodedMotorController {
	public void setAngularVelocity(double targetAngularVelocity);
	public double getAngularVelocity();
	public void setAngle(Rotation2d angle);
	public Rotation2d getAngle();
	public void setOutput(double targetPercentOutput);
	public double getPercentOutput();
	public boolean hasContinuousRotation();
	public EncodedMotorController configCurrentLimit(int currentLimitAmps);
	public EncodedMotorController configPID(PIDConstants pid);
	public EncodedMotorController configMinAngle(Rotation2d min);
	public EncodedMotorController configMaxAngle(Rotation2d max);
	public EncodedMotorController configMinOutput(double minPercentOutput);
	public EncodedMotorController configMaxOutput(double maxPercentOutput);
	public EncodedMotorController configInversion(boolean shouldInvert);
	public EncodedMotorController configBrakeOnIdle(boolean shouldBreak);
	public EncodedMotorController configAngleTolerance(Rotation2d tolerance);
}