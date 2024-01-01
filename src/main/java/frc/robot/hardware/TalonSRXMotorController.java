package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class TalonSRXMotorController extends TalonSRX implements EncodedMotorController {
    private double TICKS_PER_RADIAN = 4096 / Math.PI / 2;

    public TalonSRXMotorController(int deviceID) {
        super(deviceID);
    }

    @Override
    public void setOutput(double targetPercentOutput) {
        set(ControlMode.PercentOutput, targetPercentOutput);
    }

    @Override
    public double getPercentOutput() {
        return getMotorOutputPercent();
    }

    @Override
    public void setAngularVelocity(Rotation2d velocity) {
        set(ControlMode.Velocity, velocity.getRadians() * TICKS_PER_RADIAN / 10.0);
    }

    @Override
    public Rotation2d getAngularVelocity() {
        return Rotation2d.fromRadians(getSelectedSensorVelocity() / TICKS_PER_RADIAN * 10);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        set(ControlMode.Position, angle.getRadians() * TICKS_PER_RADIAN);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(getSelectedSensorPosition() / TICKS_PER_RADIAN);
    }

    @Override
    public boolean hasContinuousRotation() {
        return true;
    }

    @Override
    public EncodedMotorController configCurrentLimit(int currentLimit) {
        configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                true, 
                currentLimit, 
                currentLimit + 1, 
                0.1
            ), 
            50
        );
        return this;
    }

    @Override
    public EncodedMotorController configPID(PIDConstants pid) {
        config_kP(0, pid.kP);
        config_kI(0, pid.kI);
        config_kD(0, pid.kD);
        return this;
    }

    @Override
    public EncodedMotorController configMinAngle(Rotation2d min) {
        configReverseSoftLimitEnable(true);
        configReverseSoftLimitThreshold(min.getRadians() * TICKS_PER_RADIAN);
        return this;
    }

    @Override
    public EncodedMotorController configMaxAngle(Rotation2d max) {
        configForwardSoftLimitEnable(true);
        configForwardSoftLimitThreshold(max.getRadians() * TICKS_PER_RADIAN);
        return this;
    }

    @Override
    public EncodedMotorController configMinOutput(double minOutput) {
        configPeakOutputReverse(minOutput);
       return this;
    }

    @Override
    public EncodedMotorController configMaxOutput(double maxOutput) {
        configPeakOutputForward(maxOutput);
        return this;
    }
    
    @Override
    public EncodedMotorController configInversion(boolean shouldInvert) {
        setInverted(shouldInvert);
        return this;
    }

    @Override
    public EncodedMotorController configBrakeOnIdle(boolean shouldBreak) {
        setNeutralMode(
            shouldBreak
            ? NeutralMode.Brake
            : NeutralMode.Coast
        );
        return this;
    }

    @Override
    public EncodedMotorController configAngleTolerance(Rotation2d tolerance) {
        configAllowableClosedloopError(0, tolerance.getRadians() * TICKS_PER_RADIAN);
        return this;
    }
}