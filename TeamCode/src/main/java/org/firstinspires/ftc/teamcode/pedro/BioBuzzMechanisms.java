package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public final class BioBuzzMechanisms {
    public static final int LAUNCH_VELOCITY = 1150;
    private static final double LAUNCH_TRANSFER_POWER = 0.18;

    private final Servo gate;
    private final Servo flowerWedge;
    private final DcMotorEx flywheel;
    private final DcMotor intake;
    private final DcMotor transfer;

    public BioBuzzMechanisms(HardwareMap hardwareMap) {
        gate = hardwareMap.get(Servo.class, "gate");
        flowerWedge = hardwareMap.get(Servo.class, "flowerWedge");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
    }

    public void initialize() {
        setGateOpen(false);
        setFlowerWedgeLowered(false);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
    }

    void configureForAutonomous() {
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(50, 0, 0, 14.4));
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void startFeedingLauncher() {
        setTransfer(DcMotor.Direction.FORWARD, LAUNCH_TRANSFER_POWER);
        setGateOpen(true);
    }

    public void stopFeedingLauncher() {
        setGateOpen(false);
        stopTransfer();
    }

    public void setIntake(DcMotor.Direction direction, double power) {
        intake.setDirection(direction);
        intake.setPower(power);
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    public void setTransfer(DcMotor.Direction direction, double power) {
        transfer.setDirection(direction);
        transfer.setPower(power);
    }

    public void stopTransfer() {
        transfer.setPower(0);
    }

    public void setGateOpen(boolean open) {
        gate.setPosition(open ? 0.03 : 0);
    }

    public void setFlowerWedgeLowered(boolean lowered) {
        flowerWedge.setPosition(lowered ? 0.45 : 0);
    }

    public void setFlywheelVelocity(double ticksPerSecond) {
        flywheel.setVelocity(ticksPerSecond);
    }

    public void stopFlywheel() {
        flywheel.setPower(0);
    }

    public double getFlywheelVelocity() {
        return flywheel.getVelocity();
    }

    public double getFlywheelPower() {
        return flywheel.getPower();
    }
}
