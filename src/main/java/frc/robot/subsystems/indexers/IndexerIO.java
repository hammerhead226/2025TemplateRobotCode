package frc.robot.subsystems.indexers;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {

    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {}

  default void setVelocity(double velocityRadsPerSec, double ffVolts) {}

  default void configurePID(double kP, double kI, double kD) {}
}
