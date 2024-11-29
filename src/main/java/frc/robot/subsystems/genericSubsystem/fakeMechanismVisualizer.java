package frc.robot.subsystems.genericSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class fakeMechanismVisualizer {
  private final String key;
  private final Mechanism2d panel;
  private final MechanismRoot2d root;
  private final MechanismLigament2d mecha;

  public fakeMechanismVisualizer(String key, Color color) {

    this.key = key;
    this.panel = new Mechanism2d(100, 100, new Color8Bit(Color.kWhite));
    this.root = panel.getRoot("mechanism", 30, 16);
    this.mecha = root.append(new MechanismLigament2d("elevator", 5, 0, 10, new Color8Bit(color)));

    Logger.recordOutput("fakeMechanism/mechanism2d/" + key, this.panel);
  }

  public void update(double position) {
    mecha.setLength(position);
    mecha.setAngle(position);
    Logger.recordOutput("fakeMechanism/mechanism2d/" + key, this.panel);
  }
}
