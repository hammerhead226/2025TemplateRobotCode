package frc.robot.subsystems.genericSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class fakeMechanismIOTalonFX implements fakeMechanismIO {
  private static final double GEAR_RATIO = 1;

  private final TalonFX motor = new TalonFX(2);

  private final StatusSignal<Double> motorPosition = motor.getPosition();
  private final StatusSignal<Double> motorVelocity = motor.getVelocity();
  private final StatusSignal<Double> motorAppliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Double> motorCurrent = motor.getSupplyCurrent();

  private double positionSetpoint;

  public fakeMechanismIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);

    positionSetpoint = 0;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(fakeMechanismIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    inputs.positionRad = Units.rotationsToRadians(motorPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motorVelocity.getValueAsDouble() / GEAR_RATIO);
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    motor.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void setPosition(double positionRevolutions, double ffvolts) {
    positionSetpoint = positionRevolutions;
    motor.setControl(
        new PositionVoltage(positionRevolutions, 0, false, ffvolts, 0, false, false, false));
  }

  @Override
  public void stop() {
    this.positionSetpoint = motorPosition.getValueAsDouble();
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    motor.getConfigurator().apply(config);
  }
}
