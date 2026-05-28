package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@TeleOp
public class turret extends OpMode {

    private DcMotorEx turel;
    GoBildaPinpointDriver pinpointDriver;

    @Override
    public void init() {
        turel = hardwareMap.get(DcMotorEx.class, "turel");

        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint1");
        pinpointDriver.setOffsets(-15,-7.5, DistanceUnit.CM);

        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpointDriver.resetPosAndIMU();

    }

    @Override
    public void loop() {
        pinpointDriver.update();

        Pose2D pos = pinpointDriver.getPosition();

        double x = pos.getX(DistanceUnit.CM);
        double y = pos.getY(DistanceUnit.CM);

        telemetry.addData("x: ", x);
        telemetry.addData("y: ", y);
        telemetry.addData("angle: ", pinpointDriver.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}
