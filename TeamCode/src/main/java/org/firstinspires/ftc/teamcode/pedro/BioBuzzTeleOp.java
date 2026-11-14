package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

@TeleOp
public class BioBuzzTeleOp extends LinearOpMode {

    private Servo gate;
    private Servo flowerWedge;
    private DcMotor flywheel;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotor transfer;

    double currentr;
    double currentx;
    double currenty;
    int targetVelocity;
    double er;
    double ex;
    double ey;
    double rotation;
    double xpower;
    double ypower;

    /**
     * Describe this function...
     */
    @Override
    public void runOpMode() {
        int intakePower;
        double outtakePower;
        ElapsedTime flywheelTimer;
        int transferInPower;
        double transferOutPower;
        double transferLaunchPower;
        int launchTime;
        int maxVelocity;
        int flywheelThreshold;
        int flywheelTime;
        double increment1;
        double increment2;
        boolean flywheelSet;

        gate = hardwareMap.get(Servo.class, "gate");
        flowerWedge = hardwareMap.get(Servo.class, "flowerWedge");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        // intake/outtake power
        intakePower = 1;
        outtakePower = 0.5;
        // transfer power
        transferInPower = 1;
        transferOutPower = 0.5;
        // launch variables
        launchTime = 5000;
        transferLaunchPower = 0.18;
        // flywheel variables
        targetVelocity = 1150;
        maxVelocity = 1780;
        flywheelThreshold = 900;
        flywheelTime = 4000;
        increment1 = 0.5;
        increment2 = 0.8;
        flywheelSet = false;
        // timers
        flywheelTimer = new ElapsedTime();
        flywheelTimer.reset();
        // setup pinpoint odometry
        // sets the number of ticks per mm of linear travel for the odometry pod you are using
        // ticks per unit of the goBILDA 4-Bar Odometry Pod
        PinpointBlocks.encoderResolution(PinpointBlocks.FourBarOdometryPod(DistanceUnit.MM), DistanceUnit.MM);
        // Sets the odometry pod positions relative to the point that the odometry computer
        // tracks around.The X pod offset refers to how far sideways from the tracking point
        // the X (forward) odometry pod is. left is positivethe Y Pod offset refers to how far
        // forward from the tracking point the Y (strafe) odometry pod is. Forward increases
        PinpointBlocks.offsets(DistanceUnit.MM, 143.95, -90.1);
        // Setting the Boolean to true reverses the encoder, false leaves it normal
        PinpointBlocks.reverseEncoders(true, false);
        // Resets the current position to 0,0,0 and recalibrates the Odometry Computer's
        // internal IMU. Robot MUST Be stationary. Device takes a large number of samples,
        // and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
        PinpointBlocks.resetPosAndIMU();
        // motors
        gate.setPosition(0);
        flowerWedge.setPosition(0);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            ((DcMotorEx) flywheel).setVelocity(targetVelocity * increment1);
            while (opModeIsActive()) {
                // check stick locations
                ypower = -gamepad1.left_stick_y;
                xpower = gamepad1.left_stick_x;
                rotation = gamepad1.right_stick_x;
                // movement motors
                frontLeft.setPower(-(ypower + xpower + rotation));
                backLeft.setPower((ypower - xpower) + rotation);
                frontRight.setPower((ypower - xpower) - rotation);
                backRight.setPower(-((ypower + xpower) - rotation));
                // intake controls
                if (gamepad1.rightBumperWasPressed()) {
                    intake.setDirection(DcMotor.Direction.REVERSE);
                    intake.setPower(intakePower);
                    transfer.setDirection(DcMotor.Direction.FORWARD);
                    transfer.setPower(transferInPower);
                }
                if (gamepad1.rightBumperWasReleased()) {
                    intake.setPower(0);
                    transfer.setPower(0);
                }
                // outtake controls
                if (gamepad1.dpadUpWasPressed()) {
                    intake.setDirection(DcMotor.Direction.FORWARD);
                    intake.setPower(outtakePower);
                }
                if (gamepad1.dpadUpWasReleased()) {
                    intake.setPower(0);
                }
                // launch controls
                if (gamepad1.leftBumperWasPressed()) {
                    transfer.setDirection(DcMotor.Direction.FORWARD);
                    transfer.setPower(transferLaunchPower);
                    gate.setPosition(0.03);
                }
                if (gamepad1.leftBumperWasReleased()) {
                    gate.setPosition(0);
                    transfer.setPower(0);
                }
                // transfer controls
                if (gamepad1.squareWasPressed()) {
                    transfer.setDirection(DcMotor.Direction.REVERSE);
                    transfer.setPower(transferOutPower);
                }
                if (gamepad1.squareWasReleased()) {
                    transfer.setPower(0);
                }
                // flower wedge controls
                if (gamepad1.circleWasPressed()) {
                    flowerWedge.setPosition(0.45);
                }
                if (gamepad1.triangleWasPressed()) {
                    flowerWedge.setPosition(0);
                }
                // flywheel setVelocity
                if (flywheelSet == false) {
                    if (flywheelTimer.milliseconds() >= flywheelTime * 2) {
                        ((DcMotorEx) flywheel).setVelocity(targetVelocity);
                        flywheelSet = true;
                    }
                    if (flywheelTimer.milliseconds() >= flywheelTime && flywheelTimer.milliseconds() < flywheelTime * 2) {
                        ((DcMotorEx) flywheel).setVelocity(targetVelocity * increment2);
                    }
                }
                telemetry.addData("flywheelVelocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("targetVelocity", targetVelocity);
            }
        }
    }

    /**
     * Describe this function...
     */

    /**
     * Describe this function...
     */
    private void updateTelemetry2() {
        ElapsedTime loopTimer;

        // Returns the direction your robot is facing the unit of your choice
        currentr = PinpointBlocks.orientation(AngleUnit.DEGREES);
        // Returns x position the unit of your choice
        currentx = PinpointBlocks.xPosition(DistanceUnit.MM);
        // Returns y position the unit of your choice
        currenty = PinpointBlocks.yPosition(DistanceUnit.MM);
        telemetry.addLine("Pinpoint Odometry");
        telemetry.addData("currentr", currentr);
        telemetry.addData("currentx", currentx);
        telemetry.addData("currenty", currenty);
        telemetry.addLine("");
        telemetry.addLine("Motors");
        telemetry.addData("flywheelVelocity", ((DcMotorEx) flywheel).getVelocity());
        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("flywheelPower", flywheel.getPower());
        telemetry.update();
    }
}
