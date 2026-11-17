package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class BioBuzzTeleOp extends LinearOpMode {
    private static final int FLYWHEEL_STAGE_MS = 4000;
    private static final double INITIAL_VELOCITY_FRACTION = 0.5;
    private static final double INTERMEDIATE_VELOCITY_FRACTION = 0.8;

    @Override
    public void runOpMode() {
        BioBuzzMechanisms mechanisms = new BioBuzzMechanisms(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        // The original ramp counts time spent waiting for Start.
        ElapsedTime flywheelTimer = new ElapsedTime();
        flywheelTimer.reset();
        mechanisms.initialize();
        waitForStart();

        if (opModeIsActive()) {
            mechanisms.setFlywheelVelocity(BioBuzzMechanisms.LAUNCH_VELOCITY * INITIAL_VELOCITY_FRACTION);
            boolean flywheelAtTargetStage = false;
            while (opModeIsActive()) {
                drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                updateMechanismControls(mechanisms);

                if (!flywheelAtTargetStage) {
                    if (flywheelTimer.milliseconds() >= FLYWHEEL_STAGE_MS * 2) {
                        mechanisms.setFlywheelVelocity(BioBuzzMechanisms.LAUNCH_VELOCITY);
                        flywheelAtTargetStage = true;
                    }
                    if (flywheelTimer.milliseconds() >= FLYWHEEL_STAGE_MS
                            && flywheelTimer.milliseconds() < FLYWHEEL_STAGE_MS * 2) {
                        mechanisms.setFlywheelVelocity(BioBuzzMechanisms.LAUNCH_VELOCITY * INTERMEDIATE_VELOCITY_FRACTION);
                    }
                }

                updateTelemetry(mechanisms);
            }
        }
    }

    private void updateMechanismControls(BioBuzzMechanisms mechanisms) {
        // Preserve the write order when controls share a motor in the same loop.
        if (gamepad1.rightBumperWasPressed()) {
            mechanisms.setIntake(DcMotor.Direction.REVERSE, 1);
            mechanisms.setTransfer(DcMotor.Direction.FORWARD, 1);
        }
        if (gamepad1.rightBumperWasReleased()) {
            mechanisms.stopIntake();
            mechanisms.stopTransfer();
        }
        if (gamepad1.dpadUpWasPressed()) {
            mechanisms.setIntake(DcMotor.Direction.FORWARD, 0.5);
        }
        if (gamepad1.dpadUpWasReleased()) {
            mechanisms.stopIntake();
        }
        if (gamepad1.leftBumperWasPressed()) {
            mechanisms.startFeedingLauncher();
        }
        if (gamepad1.leftBumperWasReleased()) {
            mechanisms.stopFeedingLauncher();
        }
        if (gamepad1.squareWasPressed()) {
            mechanisms.setTransfer(DcMotor.Direction.REVERSE, 0.5);
        }
        if (gamepad1.squareWasReleased()) {
            mechanisms.stopTransfer();
        }
        if (gamepad1.circleWasPressed()) {
            mechanisms.setFlowerWedgeLowered(true);
        }
        if (gamepad1.triangleWasPressed()) {
            mechanisms.setFlowerWedgeLowered(false);
        }
    }

    private void updateTelemetry(BioBuzzMechanisms mechanisms) {
        telemetry.addLine("Motors");
        telemetry.addData("flywheelVelocity", mechanisms.getFlywheelVelocity());
        telemetry.addData("targetVelocity", BioBuzzMechanisms.LAUNCH_VELOCITY);
        telemetry.addData("flywheelPower", mechanisms.getFlywheelPower());
        telemetry.update();
    }
}
