package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BioBuzz Auto", group = "BioBuzz")
public class BioBuzzAuto extends OpMode {
    // Stage budgets total 27 seconds, leaving time before AUTO ends at 30 seconds.
    private static final double APPROACH_TIMEOUT_SECONDS = 4.0;
    private static final double PARKING_TIMEOUT_SECONDS = 14.0;
    private static final double AUTO_STOP_SECONDS = 29.5;
    private static final double SPINUP_TIMEOUT_SECONDS = 5.0;
    private static final double FLYWHEEL_TOLERANCE = 75;

    // Timed feed for four preloads. Tune on the robot with a full load.
    private static final double FEED_SECONDS = 4.0;

    private enum HiveRoute {
        // Estimated field coordinates in inches, origin at bottom-left, +Y upward.
        // Hive Up starts at the bottom facing up. Hive Down starts at the top facing down.
        // Each approach is six inches. Both routes park in the upper-left red LOADING ZONE.
        // Intermediate parking points take the robot around the left side of the HIVES.
        // Measure these poses and check clearance with the robot's full footprint.
        UP(new Pose(56, 9, Math.toRadians(90)), new Pose(56, 15, Math.toRadians(90)),
                new Pose(26, 24, Math.toRadians(90)),
                new Pose(26, 108, Math.toRadians(90)),
                new Pose(10, 108, Math.toRadians(90))),
        DOWN(new Pose(56, 135, Math.toRadians(-90)), new Pose(56, 129, Math.toRadians(-90)),
                new Pose(26, 129, Math.toRadians(-90)),
                new Pose(10, 108, Math.toRadians(-90)));

        final Pose start;
        final Pose launch;
        final Pose[] parkingWaypoints;

        HiveRoute(Pose start, Pose launch, Pose... parkingWaypoints) {
            this.start = start;
            this.launch = launch;
            this.parkingWaypoints = parkingWaypoints;
        }
    }

    private enum State {
        READY, APPROACH, SPINUP, FEEDING, PARKING, COMPLETE, PATH_TIMED_OUT, AUTO_TIMED_OUT
    }

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private HiveRoute route = HiveRoute.UP;
    private State state = State.READY;
    private boolean spinupTimedOut;
    private Follower follower;
    private BioBuzzMechanisms mechanisms;
    private Path approachPath;
    private Path parkingPath;

    private void buildPaths() {
        approachPath = Paths.line(route.start, route.launch).constant(route.start.heading());
        Path[] parkingSegments = new Path[route.parkingWaypoints.length];
        Pose previous = route.launch;
        for (int i = 0; i < route.parkingWaypoints.length; i++) {
            Pose destination = route.parkingWaypoints[i];
            parkingSegments[i] = Paths.line(previous, destination).constant(route.launch.heading());
            previous = destination;
        }
        parkingPath = Paths.path(parkingSegments);
    }

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
        follower.setPose(route.start);
        reportTelemetry();
    }

    @Override
    public void start() {
        buildPaths();
        autoTimer.reset();
        spinupTimedOut = false;
        follower.setPose(route.start);
        follower.follow(approachPath);
        enter(State.APPROACH);
    }

    @Override
    public void loop() {
        // Stop outputs before the 30-second AUTO ends, including if a stage runs late.
        if (autoTimer.seconds() >= AUTO_STOP_SECONDS) {
            if (state != State.COMPLETE && state != State.PATH_TIMED_OUT
                    && state != State.AUTO_TIMED_OUT) {
                finish(State.AUTO_TIMED_OUT);
            }
            reportTelemetry();
            return;
        }

        follower.update();

        switch (state) {
            case APPROACH:
                if (!follower.isBusy()) {
                    stopDrive();
                    mechanisms.setFlywheelVelocity(BioBuzzMechanisms.LAUNCH_VELOCITY);
                    enter(State.SPINUP);
                } else if (stateTimer.seconds() >= APPROACH_TIMEOUT_SECONDS) {
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
                } else if (stateTimer.seconds() >= PARKING_TIMEOUT_SECONDS) {
                    finish(State.PATH_TIMED_OUT);
                }
                break;

            case READY:
            case COMPLETE:
            case PATH_TIMED_OUT:
            case AUTO_TIMED_OUT:
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
        follower.follow(parkingPath);
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
        telemetry.addData("Launch target (in)", "%.1f, %.1f", route.launch.x(), route.launch.y());
        Pose park = route.parkingWaypoints[route.parkingWaypoints.length - 1];
        telemetry.addData("Park target (in)", "%.1f, %.1f", park.x(), park.y());
        double elapsed = state == State.READY ? 0 : autoTimer.seconds();
        telemetry.addData("AUTO seconds remaining", "%.1f", Math.max(0, 30.0 - elapsed));
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
