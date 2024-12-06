package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.PhysicalConstants;
import frc.robot.util.Conversions;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private double positionSetpoint;
  private final StatusSignal<Double> elevatorPosition;
  private final StatusSignal<Double> elevatorVelocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;

  public ElevatorIOTalonFX(int lead, int follow) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = PhysicalConstants.ElevatorConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        PhysicalConstants.ElevatorConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    leader = new TalonFX(lead, PhysicalConstants.CANBUS);
    follower = new TalonFX(follow, PhysicalConstants.CANBUS);

    leader.getConfigurator().apply(config);

    positionSetpoint = PhysicalConstants.ElevatorConstants.RETRACT_SETPOINT_INCH;

    follower.setControl(new Follower(lead, true));

    elevatorPosition = leader.getPosition();
    elevatorVelocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    currentAmps = leader.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, elevatorPosition, elevatorVelocity, appliedVolts, currentAmps);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity, appliedVolts, currentAmps);
    inputs.elevatorPosition =
        Conversions.motorRotToInches(
            elevatorPosition.getValueAsDouble(),
            5.97,
            PhysicalConstants.ElevatorConstants.REDUCTION);
    inputs.elevatorVelocity =
        Conversions.motorRotToInches(
            elevatorVelocity.getValueAsDouble() * 60.,
            5.97,
            PhysicalConstants.ElevatorConstants.REDUCTION);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.positionSetpoint = positionSetpoint;
  }

  @Override
  public void runCharacterization(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setPositionSetpoint(double position, Voltage ffVolts) {
    this.positionSetpoint = position;
    leader.setControl(
        new PositionVoltage(
            Conversions.inchesToMotorRot(
                position, 5.97, PhysicalConstants.ElevatorConstants.REDUCTION),
            0,
            false,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void stop() {
    this.positionSetpoint = elevatorPosition.getValueAsDouble();
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    leader.getConfigurator().apply(config);
  }
}
