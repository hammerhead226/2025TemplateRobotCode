package frc.robot.subsystems.arms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.PhysicalConstants;
import frc.robot.util.Conversions;
import org.littletonrobotics.junction.Logger;

public class PivotIOTalonFX implements PivotIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private final Pigeon2 pigeon;

  private double positionSetpointDegs;

  private double startAngleDegs;

  private final StatusSignal<Double> leaderPositionDegs;
  private final StatusSignal<Double> velocityDegsPerSec;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> pitch;

  public PivotIOTalonFX(int leadID, int followID, int gyroID) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = PhysicalConstants.PivotConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        PhysicalConstants.PivotConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leader = new TalonFX(leadID, PhysicalConstants.CANBUS);
    follower = new TalonFX(followID, PhysicalConstants.CANBUS);
    pigeon = new Pigeon2(gyroID, PhysicalConstants.CANBUS);
    pigeon.reset();

    leader.getConfigurator().apply(config);

    follower.setControl(new Follower(leadID, true));

    pitch = pigeon.getRoll();

    startAngleDegs = pitch.getValueAsDouble();

    leader.setPosition(
        Conversions.degreesToFalcon(startAngleDegs, PhysicalConstants.PivotConstants.REDUCTION));

    follower.setPosition(
        Conversions.degreesToFalcon(startAngleDegs, PhysicalConstants.PivotConstants.REDUCTION));

    leaderPositionDegs = leader.getPosition();
    velocityDegsPerSec = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    currentAmps = leader.getStatorCurrent();

    // leader.get

    positionSetpointDegs = PhysicalConstants.PivotConstants.STOW_SETPOINT_DEG;

    Logger.recordOutput("start angle", startAngleDegs);

    pigeon.optimizeBusUtilization();
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, leaderPositionDegs, velocityDegsPerSec, appliedVolts, currentAmps, pitch);

    // setBrakeMode(false);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPositionDegs, velocityDegsPerSec, appliedVolts, currentAmps, pitch);
    inputs.gyroConnected = BaseStatusSignal.refreshAll(pitch).equals(StatusCode.OK);
    inputs.pitch = pitch.getValueAsDouble() + PhysicalConstants.PivotConstants.PIVOT_ZERO_ANGLE;
    inputs.positionDegs =
        Conversions.falconToDegrees(
                (leaderPositionDegs.getValueAsDouble()), PhysicalConstants.PivotConstants.REDUCTION)
            + PhysicalConstants.PivotConstants.PIVOT_ZERO_ANGLE;

    // inputs.velocityDegsPerSec =
    //     Conversions.falconToDegrees(
    //         (followPositionDegs.getValueAsDouble()), Constants.PivotConstants.REDUCTION);
    inputs.velocityDegsPerSec =
        Conversions.falconToDegrees(
            velocityDegsPerSec.getValueAsDouble() * 2048,
            PhysicalConstants.PivotConstants.REDUCTION);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.positionSetpointDegs = positionSetpointDegs;
  }

  @Override
  public void setBrakeMode(boolean bool) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    if (bool) {
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
  }

  @Override
  public void setPositionSetpointDegs(double positionDegs, double ffVolts) {
    this.positionSetpointDegs = positionDegs;
    leader.setControl(
        new PositionVoltage(
            Conversions.degreesToFalcon(
                positionDegs - 59, PhysicalConstants.PivotConstants.REDUCTION),
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
    this.positionSetpointDegs = leaderPositionDegs.getValueAsDouble();
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
