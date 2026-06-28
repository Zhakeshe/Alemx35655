package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class turet extends OpMode {

    DcMotorEx turel;
    GoBildaPinpointDriver pinpointDriver;

    double kP = 0.0001;

    double goalX = 0;
    double goalY = 144;

    double TICKS_PER_REV = 537.7;
    double gearRatio = 4.0;

    double MIN_ANGLE = -160;
    double MAX_ANGLE = 160;

    @Override
    public void init() {
        turel = hardwareMap.get(DcMotorEx.class, "turel");
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpont1");

        pinpointDriver.resetPosAndIMU();
    }

    @Override
    public void loop() {

        pinpointDriver.update();

        double robotX = pinpointDriver.getPosX(DistanceUnit.INCH);
        double robotY = pinpointDriver.getPosY(DistanceUnit.INCH);
        double robotHeading = Math.toDegrees(pinpointDriver.getHeading(AngleUnit.RADIANS));

        double motorAngle = turel.getCurrentPosition() / TICKS_PER_REV * 360.0;
        double turretAngle = motorAngle / gearRatio;

        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double turretTarget = normalizeAngle(angleToGoal - robotHeading);

        double error = normalizeAngle(turretTarget - turretAngle);

        double power = error * kP;
        power = Math.max(-0.4, Math.min(0.4, power));

        if (turretAngle >= MAX_ANGLE && power > 0) {
            power = 0;
        }

        if (turretAngle <= MIN_ANGLE && power < 0) {
            power = 0;
        }

        turel.setPower(power);

        telemetry.addData("Turret Angle", turretAngle);
        telemetry.addData("Target", turretTarget);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}