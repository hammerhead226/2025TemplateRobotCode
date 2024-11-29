// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.genericSubsystem.fakeMechanism;

public class setMotorTarget extends Command {
  /** Creates a new setMotorTarget. */
  private final fakeMechanism motor;

  private double threshold;
  private double goal;

  public setMotorTarget(fakeMechanism motor, double threshold, double goal) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motor = motor;
    this.threshold = threshold;
    this.goal = goal;

    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motor.setPositionGoal(goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.motorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(motor.getMotorPosition() - goal) <= threshold;
  }
}
