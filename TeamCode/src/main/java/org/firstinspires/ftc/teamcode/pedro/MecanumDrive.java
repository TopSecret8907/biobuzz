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

    public void drive(double forward, double strafe, double turn) {
        double slowMoveSpeed = 0.5;
        double slowTurnSpeed = 0.8;
        double baseMoveSpeed = 1;
        double baseTurnSpeed = 1;
        if (BioBuzzTeleOp.slow) {
            frontLeft.setPower(forward * slowMoveSpeed + strafe * slowMoveSpeed + turn * slowTurnSpeed);
            backLeft.setPower((forward * slowMoveSpeed - strafe * slowMoveSpeed) + turn * slowTurnSpeed);
            frontRight.setPower((forward * slowMoveSpeed - strafe * slowMoveSpeed) - turn * slowTurnSpeed);
            backRight.setPower((forward * slowMoveSpeed + strafe * slowMoveSpeed) - turn * slowTurnSpeed);
        } else {
            frontLeft.setPower(forward * baseMoveSpeed + strafe * baseMoveSpeed + turn * baseTurnSpeed);
            backLeft.setPower((forward * baseMoveSpeed - strafe * baseMoveSpeed) + turn * baseTurnSpeed);
            frontRight.setPower((forward * baseMoveSpeed - strafe * baseMoveSpeed) - turn * baseTurnSpeed);
            backRight.setPower((forward * baseMoveSpeed + strafe * baseMoveSpeed) - turn * baseTurnSpeed);
        }
    }
}
