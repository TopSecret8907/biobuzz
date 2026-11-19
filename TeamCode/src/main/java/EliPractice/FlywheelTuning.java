package EliPractice;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTuning extends LinearOpMode  {
    public DcMotorEx flywheelMotor;
    // highVelocity is a constant set to the velocity we want to have when we aren't shooting
    public double highVelocity = 1150;
    // lowVelocity is a constant set to the velocity that the flywheel dips to when we shoot a ball
    public double lowVelocity = 1050;
    // curTargetVelocity is a variable set to the velocity we are trying to get to at the moment
    double curTargetVelocity = highVelocity;
    // old flywheel F = 14.4
    // old flywheel P = 50
    double F = 12.6;
    double P = 60;
    // values that get added or subtracted to F or P when we use the d-pad
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    // this tells us which value in the list we are going to use next time the d-pad is pressed
    int stepIndex = 0;

    @Override
    public void runOpMode() {
        // motor setup
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // direction is reverse on old flywheel, forward on new flywheel
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // PIDF setup
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        // ftc dashboard telemetry setup
        MultipleTelemetry tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        while (opModeIsActive()) {
            // use triangle to emulate shooting balls without shooting balls
            if (gamepad1.triangleWasPressed()) {
                if (curTargetVelocity == highVelocity) {
                    curTargetVelocity = lowVelocity;
                } else {curTargetVelocity = highVelocity;}
            }
            // change value that gets added or subtracted from P or F when using the d-pad
            if (gamepad1.circleWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            // decrease F
            if (gamepad1.dpadLeftWasPressed()) {
                F -= stepSizes[stepIndex];
            }
            // increase F
            if (gamepad1.dpadRightWasPressed()) {
                F += stepSizes[stepIndex];
            }
            // decrease P
            if (gamepad1.dpadDownWasPressed()) {
                P -= stepSizes[stepIndex];
            }
            // increase P
            if (gamepad1.dpadUpWasPressed()) {
                P += stepSizes[stepIndex];
            }

            // set new PIDF coefficients
            pidfCoefficients = new PIDFCoefficients(P,0,0,F);
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

            // set velocity
            flywheelMotor.setVelocity(curTargetVelocity);
            double curVelocity = flywheelMotor.getVelocity();
            double error = curTargetVelocity - curVelocity;
            // telemetry
            tel.addData("Target Velocity", curTargetVelocity);
            tel.addData("Current Velocity", curVelocity);
            tel.addData("Error", error);
            tel.addData("Tuning P", P);
            tel.addData("Tuning F",F);
            tel.addData("Step Size", stepSizes[stepIndex]);
            tel.update();
        }
    }
}
