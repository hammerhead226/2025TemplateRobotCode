package frc.robot.subsystems.vision;

import frc.robot.physicalConstants;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {

  public VisionIOLimelight() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.mt2VisionPose =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(physicalConstants.LL_ALIGN).pose;
    inputs.mt1VisionPose = LimelightHelpers.getBotPose2d_wpiBlue(physicalConstants.LL_ALIGN);
    inputs.timestampSeconds =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(physicalConstants.LL_ALIGN)
            .timestampSeconds;
    inputs.tagCount =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(physicalConstants.LL_ALIGN).tagCount;
    inputs.tagSpan =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(physicalConstants.LL_ALIGN).tagSpan;
    inputs.latency =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(physicalConstants.LL_ALIGN).latency;
    inputs.avgTagDist =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(physicalConstants.LL_ALIGN).avgTagDist;
    inputs.avgTagArea =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(physicalConstants.LL_ALIGN).avgTagArea;

    // Raw Limelight Data
    inputs.iTX = LimelightHelpers.getTX(physicalConstants.LL_INTAKE);
    inputs.iTY = LimelightHelpers.getTY(physicalConstants.LL_INTAKE);
    inputs.iTA = LimelightHelpers.getTA(physicalConstants.LL_INTAKE);
    inputs.iHB =
        LimelightHelpers.getLimelightNTTableEntry(physicalConstants.LL_INTAKE, "hb").getDouble(0.0);
    inputs.iTV = LimelightHelpers.getTV(physicalConstants.LL_INTAKE);
    inputs.iPIPELINELATENCY = LimelightHelpers.getLatency_Pipeline(physicalConstants.LL_INTAKE);
    inputs.iCAPTURELATENCY = LimelightHelpers.getLatency_Capture(physicalConstants.LL_INTAKE);
    inputs.iTHOR =
        LimelightHelpers.getLimelightNTTableEntry(physicalConstants.LL_INTAKE, "thor")
            .getDouble(0.0);
    inputs.iTVERT =
        LimelightHelpers.getLimelightNTTableEntry(physicalConstants.LL_INTAKE, "tvert")
            .getDouble(0.0);

    inputs.aTX = LimelightHelpers.getTX(physicalConstants.LL_ALIGN);
    inputs.aTY = LimelightHelpers.getTY(physicalConstants.LL_ALIGN);
    inputs.aTA = LimelightHelpers.getTA(physicalConstants.LL_ALIGN);
    inputs.aHB =
        LimelightHelpers.getLimelightNTTableEntry(physicalConstants.LL_ALIGN, "hb").getDouble(0.0);
    inputs.aTV = LimelightHelpers.getTV(physicalConstants.LL_ALIGN);
    inputs.aPIPELINELATENCY = LimelightHelpers.getLatency_Pipeline(physicalConstants.LL_ALIGN);
    inputs.aCAPTURELATENCY = LimelightHelpers.getLatency_Capture(physicalConstants.LL_ALIGN);
    inputs.aTHOR =
        LimelightHelpers.getLimelightNTTableEntry(physicalConstants.LL_ALIGN, "thor")
            .getDouble(0.0);
    inputs.aTVERT =
        LimelightHelpers.getLimelightNTTableEntry(physicalConstants.LL_ALIGN, "tvert")
            .getDouble(0.0);
  }
}
