package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class MecanumDrive {
    private final DcMotor frontLeft;
    private final DcMotor backLeft;
    private final DcMotor frontRight;
    private final DcMotor backRight;

    public MecanumDrive(HardwareMap hardwareMap) {
        MecanumConfig config = BiobuzzConstants.mecanumConfig;
        frontLeft = hardwareMap.get(DcMotor.class, config.frontLeftName.get());
        backLeft = hardwareMap.get(DcMotor.class, config.backLeftName.get());
        frontRight = hardwareMap.get(DcMotor.class, config.frontRightName.get());
        backRight = hardwareMap.get(DcMotor.class, config.backRightName.get());

        frontLeft.setDirection(config.frontLeftDirection.get());
        backLeft.setDirection(config.backLeftDirection.get());
        frontRight.setDirection(config.frontRightDirection.get());
        backRight.setDirection(config.backRightDirection.get());
    }

    /** Applies the signed wheel mix from the original TeleOp. */
    public void drive(double forward, double strafe, double turn) {
        frontLeft.setPower(-(forward + strafe + turn));
        backLeft.setPower((forward - strafe) + turn);
        frontRight.setPower((forward - strafe) - turn);
        backRight.setPower(-((forward + strafe) - turn));
    }
}
