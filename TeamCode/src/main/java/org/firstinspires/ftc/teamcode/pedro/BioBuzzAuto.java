package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Java/Pedro replacement for the Blocks-based moveToPoint approach.
 */
@Autonomous(name = "BioBuzz Auto (route template)", group = "BioBuzz")
public class BioBuzzAuto extends OpMode {
    private static final double SEGMENT_TIMEOUT_SECONDS = 8.0;

    // Servo positions & timing constants
    public static double GATE_DOWN_POSITION = 0.0;
    public static double GATE_UP_POSITION = 1.0;
    public static double FLOWER_WEDGE_DEFAULT_POSITION = 0.0;
    public static double GATE_UP_DURATION_SECONDS = 1.5;

    // Default motor powers
    public static double INTAKE_POWER = 1.0;
    public static double FLYWHEEL_POWER = 0.85;
    public static double TRANSFER_POWER = 1.0;

    // Placeholder origin: replace with the actual field starting pose.
    // All route coordinates are inches; all headings are radians (CCW positive).
    private static final Pose START_POSE = new Pose(0, 0, Math.toRadians(0));

    private enum State {READY, FOLLOWING, COMPLETE, TIMED_OUT}

    private final List<Path> route = new ArrayList<>();
    private final ElapsedTime segmentTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();

    private Follower follower;
    private State state = State.READY;
    private int segmentIndex;

    // Expansion Hub Motors
    private DcMotorEx intake;    // Port 0 (Expansion Hub)
    public DcMotorEx flywheelMotor;
    private DcMotorEx transfer;  // Port 2 (Expansion Hub)

    // Control Hub Servos
    private Servo gate;          // Port 0 (Control Hub)
    private Servo flowerWedge;   // Port 1 (Control Hub)

    private boolean gateIsUp = false;


    /**
     * Add destinations in order once the route is known. Empty means no movement.
     */
    private void buildRoute() {
        route.clear();
        Pose previous = START_POSE;
        for (Pose destination : routeWaypoints()) {
            Path path = Paths.line(previous, destination);
            route.add(path);
            previous = destination;
        }
    }

    private Pose[] routeWaypoints() {

        return new Pose[]{
                new Pose(0, -56, Math.toRadians(90)),
                new Pose(0, -46, Math.toRadians(90))
        };

    }


    @Override
    public void init() {
        follower = BiobuzzConstants.create(hardwareMap);
        follower.setPose(START_POSE);

        // HardwareMap setup for Expansion Hub motors
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(50,0,0,14.4);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        flywheelMotor.setVelocity(1150);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // HardwareMap setup for Control Hub servos
        gate = hardwareMap.get(Servo.class, "gate");
        flowerWedge = hardwareMap.get(Servo.class, "flowerWedge");

        // Initial servo positions
        setGateDown();
        flowerWedge.setPosition(FLOWER_WEDGE_DEFAULT_POSITION);

        buildRoute();
        reportTelemetry();
    }

    @Override
    public void init_loop() {
        // Update localization before START without commanding a path.
        follower.update();
        reportTelemetry();
    }

    @Override
    public void start() {
        segmentIndex = 0;
        if (route.isEmpty()) {
            finish(State.COMPLETE);
        } else {
            startSegment();
        }
    }

    private void startSegment() {
        follower.follow(route.get(segmentIndex));
        segmentTimer.reset();
        state = State.FOLLOWING;
    }

    @Override
    public void loop() {
        follower.update(); // Required every loop, including while following a path.

        // Automatically reset the gate servo after the set duration when transfer is active
        if (gateIsUp && gateTimer.seconds() >= GATE_UP_DURATION_SECONDS) {
            setGateDown();
        }

        if (state == State.FOLLOWING) {
            if (segmentTimer.seconds() >= SEGMENT_TIMEOUT_SECONDS) {
                finish(State.TIMED_OUT);
            } else if (!follower.isBusy()) {
                segmentIndex++;
                if (segmentIndex < route.size()) {
                    startSegment();
                } else {
                    finish(State.COMPLETE);
                }
            }
        }
        reportTelemetry();
    }

    // --- Component Helper Methods ---

    public void startIntake(double power) {
        if (intake != null) intake.setPower(power);
    }

    public void stopIntake() {
        if (intake != null) intake.setPower(0);
    }

    public void startFlywheel(double power) {
        if (flywheelMotor != null) flywheelMotor.setPower(power);
    }

    public void stopFlywheel() {
        if (flywheelMotor != null) flywheelMotor.setPower(0);
    }

    /**
     * Starts transfer and raises the gate servo immediately.
     */
    public void startTransfer(double power) {
        if (transfer != null) transfer.setPower(power);
        if (power > 0) {
            setGateUp();
        } else {
            setGateDown();
        }
    }

    public void stopTransfer() {
        if (transfer != null) transfer.setPower(0);
        setGateDown();
    }

    public void setGateUp() {
        if (gate != null) gate.setPosition(GATE_UP_POSITION);
        gateTimer.reset();
        gateIsUp = true;
    }

    public void setGateDown() {
        if (gate != null) gate.setPosition(GATE_DOWN_POSITION);
        gateIsUp = false;
    }

    public void setFlowerWedgePosition(double position) {
        if (flowerWedge != null) flowerWedge.setPosition(position);
    }

    private void finish(State result) {
        follower.stop();
        state = result;
    }

    private void reportTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("State", state);
        telemetry.addData("Route segments", route.size());
        telemetry.addData("Segment index (zero based)", segmentIndex);
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));

        // Hardware Telemetry
        telemetry.addData("Intake Power", intake != null ? intake.getPower() : 0);
        telemetry.addData("Flywheel Power", flywheelMotor != null ? flywheelMotor.getPower() : 0);
        telemetry.addData("Transfer Power", transfer != null ? transfer.getPower() : 0);
        telemetry.addData("Gate Pos", gate != null ? gate.getPosition() : 0);
        telemetry.addData("Gate Is Up", gateIsUp);
        telemetry.addData("FlowerWedge Pos", flowerWedge != null ? flowerWedge.getPosition() : 0);

        if (route.isEmpty()) {
            telemetry.addData("Route", "No waypoints configured; robot will stay still");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
        }
        stopIntake();
        stopFlywheel();
        stopTransfer();
        setGateDown();
    }
}
