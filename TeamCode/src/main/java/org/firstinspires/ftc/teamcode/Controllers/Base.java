package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BBQ.PIDF;

@Config
public class Base {
    private double targetHeading = 0;
    private PIDF headingPIDF;

    public static double Kp = 1.5;
    public static double Ki = 0;
    public static double Kd = 0.1;
    public static double Kf = 0;
    public static double ROTATION_MULTIPLIER = 0.65;

    private DcMotor Lfront, Rfront, Rback, Lback;
    private GoBildaPinpointDriver pinpoint;

    public void initialize(HardwareMap hardwareMap, boolean useBrakeMode) {
        headingPIDF = new PIDF(Kp, Ki, Kd, Kf);

        Lfront = hardwareMap.get(DcMotor.class, "lf");
        Rfront = hardwareMap.get(DcMotor.class, "rf");
        Lback = hardwareMap.get(DcMotor.class, "lr");
        Rback = hardwareMap.get(DcMotor.class, "rr");

        Lfront.setDirection(DcMotorSimple.Direction.REVERSE);
        Lback.setDirection(DcMotorSimple.Direction.REVERSE);
        Rfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Rback.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor.ZeroPowerBehavior behavior = useBrakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        Lfront.setZeroPowerBehavior(behavior);
        Rfront.setZeroPowerBehavior(behavior);
        Lback.setZeroPowerBehavior(behavior);
        Rback.setZeroPowerBehavior(behavior);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public static double fieldCentricOffset = 0;

    public void update(double x, double y, double rx, double turnCoeff, boolean headingLock, boolean isRobotCentric) {
        pinpoint.update();
        double botHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        double limitedRx = rx * ROTATION_MULTIPLIER;

        if (headingLock) {
            limitedRx = headingPIDF.calculate(botHeading);
            limitedRx = Math.min(Math.max(limitedRx, -ROTATION_MULTIPLIER), ROTATION_MULTIPLIER);
        }

        if (isRobotCentric) {
            drive(x, y, limitedRx);
        } else {
            double fcHeading = botHeading - fieldCentricOffset;
            double rotX = x * Math.cos(-fcHeading) - y * Math.sin(-fcHeading);
            double rotY = x * Math.sin(-fcHeading) + y * Math.cos(-fcHeading);
            drive(rotX, rotY, limitedRx);
        }
    }

    private void drive(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        Lfront.setPower(frontLeftPower);
        Lback.setPower(backLeftPower);
        Rfront.setPower(frontRightPower);
        Rback.setPower(backRightPower);
    }

    public void resetHeading(double angle) {
        pinpoint.setHeading(angle, AngleUnit.RADIANS);
    }

    public void setTargetHeading(double angle) {
        targetHeading = angle;
        headingPIDF.setSetPoint(targetHeading);
    }
}