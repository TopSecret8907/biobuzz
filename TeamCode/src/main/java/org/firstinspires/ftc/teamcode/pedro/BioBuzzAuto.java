package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BioBuzz Auto", group = "BioBuzz")
public class BioBuzzAuto extends OpMode {
    private static final double PATH_TIMEOUT_SECONDS = 8.0;
    private static final double SPINUP_TIMEOUT_SECONDS = 5.0;
    private static final double FLYWHEEL_TOLERANCE = 75;

    // Timed feed for four preloads. Tune on the robot with a full load.
    private static final double FEED_SECONDS = 4.0;

    private enum HiveRoute {
        // Placeholder poses in inches and radians. Replace with measured field poses.
        UP(new Pose(0, 0, 0), new Pose(12, 0, Math.toRadians(5)), new Pose(12, 12, 0)),
        DOWN(new Pose(0, 0, 0), new Pose(12, 0, Math.toRadians(-5)), new Pose(12, -12, 0));

        final Pose start;
        final Pose launch;
        final Pose park;

        HiveRoute(Pose start, Pose launch, Pose park) {
            this.start = start;
            this.launch = launch;
            this.park = park;
        }
    }

    private enum State {READY, APPROACH, SPINUP, FEEDING, PARKING, COMPLETE, PATH_TIMED_OUT}

    private final ElapsedTime stateTimer = new ElapsedTime();
    private HiveRoute route = HiveRoute.UP;
    private State state = State.READY;
    private boolean spinupTimedOut;
    private Follower follower;
    private BioBuzzMechanisms mechanisms;

    @Override
    public void init() {
        follower = BiobuzzConstants.create(hardwareMap);
        follower.setPose(route.start);
        mechanisms = new BioBuzzMechanisms(hardwareMap);
        mechanisms.initialize();
        mechanisms.configureForAutonomous();
        stopLauncher();
        mechanisms.stopIntake();
        reportTelemetry();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) {
            route = HiveRoute.UP;
        } else if (gamepad1.dpad_down) {
            route = HiveRoute.DOWN;
        }

        follower.update();
        reportTelemetry();
    }

    @Override
    public void start() {
        follower.setPose(route.start);
        follower.follow(Paths.line(route.start, route.launch));
        enter(State.APPROACH);
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case APPROACH:
                if (!follower.isBusy()) {
                    stopDrive();
                    mechanisms.setFlywheelVelocity(BioBuzzMechanisms.LAUNCH_VELOCITY);
                    enter(State.SPINUP);
                } else if (stateTimer.seconds() >= PATH_TIMEOUT_SECONDS) {
                    finish(State.PATH_TIMED_OUT);
                }
                break;

            case SPINUP:
                double speedError = Math.abs(BioBuzzMechanisms.LAUNCH_VELOCITY
                        - mechanisms.getFlywheelVelocity());
                if (speedError <= FLYWHEEL_TOLERANCE) {
                    mechanisms.startFeedingLauncher();
                    enter(State.FEEDING);
                } else if (stateTimer.seconds() >= SPINUP_TIMEOUT_SECONDS) {
                    spinupTimedOut = true;
                    startParking();
                }
                break;

            case FEEDING:
                if (stateTimer.seconds() >= FEED_SECONDS) {
                    startParking();
                }
                break;

            case PARKING:
                if (!follower.isBusy()) {
                    finish(State.COMPLETE);
                } else if (stateTimer.seconds() >= PATH_TIMEOUT_SECONDS) {
                    finish(State.PATH_TIMED_OUT);
                }
                break;

            case READY:
            case COMPLETE:
            case PATH_TIMED_OUT:
                break;
        }

        reportTelemetry();
    }

    private void enter(State nextState) {
        state = nextState;
        stateTimer.reset();
    }

    private void startParking() {
        stopLauncher();
        follower.follow(Paths.line(route.launch, route.park));
        enter(State.PARKING);
    }

    private void finish(State result) {
        stopDrive();
        stopLauncher();
        enter(result);
    }

    private void stopDrive() {
        follower.stop();
        follower.drivetrain.stop();
    }

    private void stopLauncher() {
        mechanisms.stopFeedingLauncher();
        mechanisms.stopFlywheel();
    }

    private void reportTelemetry() {
        telemetry.addData("Hive route", route);
        if (state == State.READY) {
            telemetry.addLine("D-pad up/down selects the hive route");
        }
        telemetry.addData("State", state);
        if (spinupTimedOut) {
            telemetry.addLine("Flywheel speed timed out. Launch skipped.");
        }
        Pose pose = follower.pose();
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("Flywheel velocity", mechanisms.getFlywheelVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            stopDrive();
        }
        if (mechanisms != null) {
            stopLauncher();
            mechanisms.stopIntake();
        }
    }
}
