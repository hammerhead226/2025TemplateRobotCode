package frc.robot.subsystems.indexers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX talon;

  private final StatusSignal<Double> positionRads;
  private final StatusSignal<Double> velocityRadsPerSec;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;

  public IndexerIOTalonFX(int motorID) {
    talon = new TalonFX(motorID);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.CurrentLimits.SupplyCurrentLimit = 30.0;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talon.getConfigurator().apply(configs);

    positionRads = talon.getPosition();
    velocityRadsPerSec = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    currentAmps = talon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRads, velocityRadsPerSec, appliedVolts, currentAmps);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRads, velocityRadsPerSec, appliedVolts, currentAmps);

    inputs.positionRads = positionRads.getValueAsDouble();
    inputs.velocityRadsPerSec = velocityRadsPerSec.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setPosition(double positionDegs, double ffVolts) {
    talon.setControl(
        new PositionVoltage(
            Units.degreesToRotations(positionDegs), 0, false, ffVolts, 0, false, false, false));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    talon.getConfigurator().apply(config);
  }
}
