package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public enum LimelightPipeline {

    UNKNOWN(-1),
    DEFAULT_APRILTAG(0),
    GAMEPIECE(1);

    public final int id;

    LimelightPipeline(int id) {
        this.id = id;
    }

    public static LimelightPipeline forId(int id) {
        for (LimelightPipeline pipeline : values()) {
            if (id == pipeline.id) {
                return pipeline;
            }
        }
        return UNKNOWN;
    }

    public static SendableChooser<LimelightPipeline> makeChooser() {
        SendableChooser<LimelightPipeline> chooser = new SendableChooser<>();
        LimelightPipeline [] pipelines = values();
        for (int i=0; i<pipelines.length; i++) {
            if (i == 0) {
                chooser.setDefaultOption(pipelines[i].name(), pipelines[i]);
            } else {
                chooser.addOption(pipelines[i].name(), pipelines[i]);
            }
        }
        return chooser;
    }
}
