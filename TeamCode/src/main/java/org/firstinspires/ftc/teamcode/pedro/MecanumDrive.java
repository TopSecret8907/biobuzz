package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class MecanumDrive {
    private final Mecanum drivetrain;

    public MecanumDrive(HardwareMap hardwareMap) {
        drivetrain = BiobuzzConstants.createDrivetrain(hardwareMap);
    }

    public void drive(double forward, double strafe, double turn) {
        // Convert the TeleOp joystick signs to Pedro's strafe and turn conventions.
        drivetrain.applyDrive(new DrivePowers(forward, -strafe, -turn));
    }
}
