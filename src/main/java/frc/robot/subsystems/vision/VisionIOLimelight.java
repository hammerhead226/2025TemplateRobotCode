package frc.robot.subsystems.vision;

import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {

  String name;

  public VisionIOLimelight(String name, String name2) {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.mt2VisionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose;
    inputs.mt1VisionPose = LimelightHelpers.getBotPose2d_wpiBlue(name);
    inputs.timestampSeconds =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).timestampSeconds;
    inputs.tagCount = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).tagCount;
    inputs.tagSpan = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).tagSpan;
    inputs.latency = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).latency;
    inputs.avgTagDist = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).avgTagDist;
    inputs.avgTagArea = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).avgTagArea;

    // --------------------------

    inputs.iTX = LimelightHelpers.getTX(name);
    inputs.iTY = LimelightHelpers.getTY(name);
    inputs.iTA = LimelightHelpers.getTA(name);
    inputs.iHB = LimelightHelpers.getLimelightNTTableEntry(name, "hb").getDouble(0.0);
    inputs.iTV = LimelightHelpers.getTV(name);
    inputs.iPIPELINELATENCY = LimelightHelpers.getLatency_Pipeline(name);
    inputs.iCAPTURELATENCY = LimelightHelpers.getLatency_Capture(name);
    inputs.iTHOR = LimelightHelpers.getLimelightNTTableEntry(name, "thor").getDouble(0.0);
    inputs.iTVERT = LimelightHelpers.getLimelightNTTableEntry(name, "tvert").getDouble(0.0);

    // --------------------------

    inputs.aTX = LimelightHelpers.getTX(name);
    inputs.aTY = LimelightHelpers.getTY(name);
    inputs.aTA = LimelightHelpers.getTA(name);
    inputs.aHB = LimelightHelpers.getLimelightNTTableEntry(name, "hb").getDouble(0.0);
    inputs.aTV = LimelightHelpers.getTV(name);
    inputs.aPIPELINELATENCY = LimelightHelpers.getLatency_Pipeline(name);
    inputs.aCAPTURELATENCY = LimelightHelpers.getLatency_Capture(name);
    inputs.aTHOR = LimelightHelpers.getLimelightNTTableEntry(name, "thor").getDouble(0.0);
    inputs.aTVERT = LimelightHelpers.getLimelightNTTableEntry(name, "tvert").getDouble(0.0);
  }
}
