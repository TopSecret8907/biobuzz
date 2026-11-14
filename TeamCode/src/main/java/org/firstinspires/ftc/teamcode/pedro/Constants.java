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

public class Constants {
    public static final MecanumConfig mecanumConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.backLeftName.set("backLeft");
        c.frontRightName.set("frontRight");
        c.backRightName.set("backRight");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
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
        Mecanum drivetrain = new Mecanum(h, mecanumConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(h, pinpointConfig);
        Foresight algorithm = new Foresight(foresightConfig);
        return new Follower(localizer, drivetrain, algorithm);
    }
}