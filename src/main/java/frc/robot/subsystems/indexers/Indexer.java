// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.physicalConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final IndexerIO indexer;

  private final IndexerIOInputsAutoLogged Iinputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO indexer) {
    this.indexer = indexer;

    switch (physicalConstants.currentMode) {
      case REAL:
      case REPLAY:
        break;
      case SIM:
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    indexer.updateInputs(Iinputs);
  }
}
