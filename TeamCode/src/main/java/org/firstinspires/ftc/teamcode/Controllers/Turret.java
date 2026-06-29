package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Turret {

    public enum Mode { ODOMETRY, FIELD_45, FIELD_NEG_45, DEGREE_LOCK, MANUAL, DISABLED }

    private Mode currentMode = Mode.MANUAL;

    private Servo turretr;
    private Servo turretl;

    public static double MIN_ANGLE = -85.0;
    public static double MAX_ANGLE =  85.0;

    private double targetAngle      = 0;
    private double lockedFieldAngle = 0;

    public void init(HardwareMap hardwareMap) {
        turretr = hardwareMap.get(Servo.class, "turel");
        turretl = hardwareMap.get(Servo.class, "turel1");
    }

    public void update(Pose robotPose) {
        if (currentMode == Mode.DISABLED) {
            return;
        }

        switch (currentMode) {
            case ODOMETRY: {
                Pose target = Action.getActiveTarget();
                targetAngle = calculateOdoAngle(robotPose, target.getX(), target.getY());
                break;
            }

            case FIELD_45:
                targetAngle = calculateFixedFieldAngle(robotPose, 45);
                break;

            case FIELD_NEG_45:
                targetAngle = calculateFixedFieldAngle(robotPose, -45);
                break;

            case DEGREE_LOCK:
                targetAngle = lockedFieldAngle;
                break;

            case MANUAL:
                return;
        }

        targetAngle = Math.max(MIN_ANGLE, Math.min(targetAngle, MAX_ANGLE));
        setAngle(targetAngle);
    }

    public void setDegreeLock(double fieldAngleDeg) {
        this.lockedFieldAngle = fieldAngleDeg;
        setMode(Mode.DEGREE_LOCK);
    }

    public void setManualAngle(double angleDeg) {
        if (currentMode != Mode.MANUAL) return;
        targetAngle = Math.max(MIN_ANGLE, Math.min(angleDeg, MAX_ANGLE));
        setAngle(targetAngle);
    }

    public void setMode(Mode mode) {
        this.currentMode = mode;
    }

    public double getTargetAngle() { return targetAngle; }

    /**
     * Перевод угла в позицию серво.
     * 0.5 -> 0°
     * 0.0 -> 85°
     * 1.0 -> -85°
     */
    private void setAngle(double angleDeg) {
        angleDeg = Math.max(MIN_ANGLE, Math.min(angleDeg, MAX_ANGLE));

        double pos = 0.5 - (angleDeg / 170.0);

        pos = Math.max(0.0, Math.min(pos, 1.0));

        turretr.setPosition(pos);
        turretl.setPosition(pos);
    }

    private double calculateOdoAngle(Pose robotPose, double tx, double ty) {
        double dx = tx - robotPose.getX();
        double dy = ty - robotPose.getY();
        double angleToTarget = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading  = Math.toDegrees(robotPose.getHeading());
        return angleToTarget - robotHeading;
    }

    private double calculateFixedFieldAngle(Pose robotPose, double fieldAngle) {
        double robotHeading = Math.toDegrees(robotPose.getHeading());
        return fieldAngle - robotHeading;
    }
}