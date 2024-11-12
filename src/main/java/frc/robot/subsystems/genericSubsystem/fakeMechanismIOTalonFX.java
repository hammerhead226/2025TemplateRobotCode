package frc.robot.subsystems.genericSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class fakeMechanismIOTalonFX implements fakeMechanismIO {
  private static final double GEAR_RATIO = 1;

  private final TalonFX motor = new TalonFX(0);

  private final StatusSignal<Double> motorPosition = motor.getPosition();
  private final StatusSignal<Double> motorVelocity = motor.getVelocity();
  private final StatusSignal<Double> motorAppliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Double> motorCurrent = motor.getSupplyCurrent();

  public fakeMechanismIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    motor.optimizeBusUtilization();
  }
//provide implementation for the updateInputs method
//DONT FORGET TO REFRESH THE SIGNAL HERE!
  @Override
  public void updateInputs(fakeMechanismIOInputs inputs) {
   
  }
  //provide implementation for the setVoltage method 
  //the motor class has its own setVoltage method, you can use that 
  @Override
  public void setVoltage(double volts) {
   
  }
//finish the implementation for the setVelocity method 
//most of it is filled out for you, but you need to figure out how to use the parameters given 
//there are only two blanks, if you hover over the VelocityVoltage object then you can see what each parameter should be
//BE CAREFUL WITH UNITS, what kind of velocity does the object take in?
  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    motor.setControl(
        new VelocityVoltage(
            ???,
            0.0,
            true,
            ???,
            0,
            false,
            false,
            false));
  }

  //provide the implementation for the stop method 
  @Override
  public void stop() {
    
  }
//provide the implementation for the configure PID method 
//the config object has its own kP kI and kD values that you must set to the parameters 
//we set pid this way because the motor has its own pid controller

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
   
    motor.getConfigurator().apply(config);
  }
}
