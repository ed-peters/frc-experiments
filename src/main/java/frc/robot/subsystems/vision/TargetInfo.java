package frc.robot.subsystems.vision;

public class TargetInfo {

    public boolean hasTarget;
    public double horizontalOffset;
    public double verticalOffset;
    public double targetArea;

    public void setNoTarget() {
        this.hasTarget = false;
        this.horizontalOffset = 0.0;
        this.verticalOffset = 0.0;
        this.targetArea = 0.0;
    }

    public void setTarget(double horizontalOffset, double verticalOffset, double targetArea) {
        this.hasTarget = true;
        this.horizontalOffset = horizontalOffset;
        this.verticalOffset = verticalOffset;
        this.targetArea = targetArea;
    }
}
