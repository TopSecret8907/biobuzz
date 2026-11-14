package org.firstinspires.ftc.teamcode.pedro.config;

import static org.firstinspires.ftc.teamcode.pedro.config.ConfigFile.goalHeight;
import static org.firstinspires.ftc.teamcode.pedro.config.ConfigFile.limelightHeight;
import static org.firstinspires.ftc.teamcode.pedro.config.ConfigFile.limelightOffsetForward;
import static org.firstinspires.ftc.teamcode.pedro.config.ConfigFile.limelightOffsetRightwards;

public class LimelightHeadingCorrection {

    //tx=deg, ty=deg, x=inch, y=inch, heading=deg, ID=int

    public static double LimelightCorrection(double tx, double ty, double x, double y, double heading, int ID) {
        double limelightDistance = (goalHeight - limelightHeight) / Math.tan(Math.toRadians(ty));
        double deltaRightwardsLimelight = Math.sin(Math.toRadians(tx)) * limelightDistance;
        double deltaForwardsLimelight = Math.cos(Math.toRadians(tx)) * limelightDistance;
        double ROBOtx = 90 - Math.toDegrees(Math.atan2(deltaForwardsLimelight + limelightOffsetForward, deltaRightwardsLimelight + limelightOffsetRightwards));
        double robotXYdeltaThetaGoal;

        if (ID == 20) {
            robotXYdeltaThetaGoal = Math.toDegrees(Math.atan2(133-y, 17-x));
        } else if (ID == 24) {
            robotXYdeltaThetaGoal = Math.toDegrees(Math.atan2(133-y, 127-x));
        } else {
            return heading;
        }
        return robotXYdeltaThetaGoal+ROBOtx;
    }
}
