package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BiobuzzConstants {
    public static final MecanumConfig mecanumConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.backLeftName.set("backLeft");
        c.frontRightName.set("frontRight");
        c.backRightName.set("backRight");

        // TeleOp uses forward motor directions and applies wheel signs in its power mix.
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static final PinpointConfig pinpointConfig = new PinpointConfig(c -> {
        c.name.set("odometry");
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.xPodOffset.set(143.95);
        c.yPodOffset.set(-90.1);
        c.offsetUnits.set(DistanceUnit.MM);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    });

    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {});

    public static Follower create(HardwareMap h) {
        // Pedro's wheel mix omits the front-left/back-right signs used by TeleOp.
        // Apply those signs as motor directions for the path follower.
        MecanumConfig pedroConfig = new MecanumConfig(c -> {
            c.frontLeftName.set(mecanumConfig.frontLeftName.get());
            c.backLeftName.set(mecanumConfig.backLeftName.get());
            c.frontRightName.set(mecanumConfig.frontRightName.get());
            c.backRightName.set(mecanumConfig.backRightName.get());
            c.frontLeftDirection.set(mecanumConfig.frontLeftDirection.get().inverted());
            c.backLeftDirection.set(mecanumConfig.backLeftDirection.get());
            c.frontRightDirection.set(mecanumConfig.frontRightDirection.get());
            c.backRightDirection.set(mecanumConfig.backRightDirection.get().inverted());
            c.manualBrakeMode.set(mecanumConfig.manualBrakeMode.get());
            c.powerThreshold.set(mecanumConfig.powerThreshold.get());
        });
        Mecanum drivetrain = new Mecanum(h, pedroConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(h, pinpointConfig);
        Foresight algorithm = new Foresight(foresightConfig);
        return new Follower(localizer, drivetrain, algorithm);
    }
}