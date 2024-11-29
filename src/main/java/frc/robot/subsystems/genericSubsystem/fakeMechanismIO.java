package frc.robot.subsystems.genericSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface fakeMechanismIO {
  @AutoLog
  public static class fakeMechanismIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(fakeMechanismIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}

  /** create a set position method here * */
  // should have a double parameter called goalRevolutions and feedforward volts

  public default void setPosition(double goalRevolutions, double volts) {}
}
