package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Turel Dual Servo", group = "Digital")
public class TurelDualServoTeleOp extends OpMode {

    public static double CENTER_POSITION = 0.50;
    public static double MIN_POSITION = 0.00;
    public static double MAX_POSITION = 1.00;

    public static double STEP_DEGREES = 10.0;
    public static double FINE_STEP_DEGREES = 2.0;
    public static double HOLD_START_DELAY = 0.30;
    public static double HOLD_SPEED_DEG_PER_SEC = 120.0;
    public static double STICK_SPEED_DEG_PER_SEC = 180.0;
    public static double MAX_SERVO_MOVE_PER_SEC = 1.8;
    public static double STICK_DEADZONE = 0.10;
    public static boolean MIRROR_SECOND_SERVO = false;
    public static double POSITION_PER_DEGREE = 1.0 / 300.0;

    private Servo turel;
    private Servo turel1;

    private double targetPosition = CENTER_POSITION;
    private double appliedPosition = CENTER_POSITION;
    private double lastLoopTime = 0.0;

    private boolean leftWasPressed = false;
    private boolean rightWasPressed = false;
    private boolean upWasPressed = false;
    private boolean downWasPressed = false;
    private boolean aWasPressed = false;
    private double leftHoldStart = 0.0;
    private double rightHoldStart = 0.0;

    @Override
    public void init() {
        turel = hardwareMap.get(Servo.class, "turel");
        turel1 = hardwareMap.get(Servo.class, "turel1");

        targetPosition = Range.clip(turel.getPosition(), MIN_POSITION, MAX_POSITION);
        appliedPosition = targetPosition;
        applyPosition(appliedPosition);
        lastLoopTime = getRuntime();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double deltaTime = Math.max(0.0, now - lastLoopTime);
        lastLoopTime = now;

        boolean leftPressed = gamepad1.dpad_left;
        boolean rightPressed = gamepad1.dpad_right;
        boolean upPressed = gamepad1.dpad_up;
        boolean downPressed = gamepad1.dpad_down;
        boolean aPressed = gamepad1.a;
        double stick = applyDeadzone(gamepad1.left_stick_x);

        if (leftPressed && !leftWasPressed) {
            stepDegrees(-STEP_DEGREES);
            leftHoldStart = now;
        }

        if (rightPressed && !rightWasPressed) {
            stepDegrees(STEP_DEGREES);
            rightHoldStart = now;
        }

        if (upPressed && !upWasPressed) {
            stepDegrees(FINE_STEP_DEGREES);
        }

        if (downPressed && !downWasPressed) {
            stepDegrees(-FINE_STEP_DEGREES);
        }

        if (aPressed && !aWasPressed) {
            targetPosition = Range.clip(CENTER_POSITION, MIN_POSITION, MAX_POSITION);
        }

        if (leftPressed && now - leftHoldStart >= HOLD_START_DELAY) {
            stepDegrees(-HOLD_SPEED_DEG_PER_SEC * deltaTime);
        }

        if (rightPressed && now - rightHoldStart >= HOLD_START_DELAY) {
            stepDegrees(HOLD_SPEED_DEG_PER_SEC * deltaTime);
        }

        if (Math.abs(stick) > 0.0) {
            stepDegrees(stick * STICK_SPEED_DEG_PER_SEC * deltaTime);
        }

        leftWasPressed = leftPressed;
        rightWasPressed = rightPressed;
        upWasPressed = upPressed;
        downWasPressed = downPressed;
        aWasPressed = aPressed;

        appliedPosition = moveTowards(appliedPosition, targetPosition, MAX_SERVO_MOVE_PER_SEC * deltaTime);
        applyPosition(appliedPosition);

        telemetry.addLine("ZHORIK TOP");
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Applied Position", appliedPosition);
        telemetry.addData("Approx Degrees From Center", (targetPosition - CENTER_POSITION) / POSITION_PER_DEGREE);
        telemetry.addData("Stick X", stick);
        telemetry.addData("Mirror Second Servo", MIRROR_SECOND_SERVO);
        telemetry.addLine("dpad L/R = 10 deg, hold = continuous");
        telemetry.addLine("dpad U/D = fine step, A = center, left stick = analog");
        telemetry.update();
    }

    private void stepDegrees(double degrees) {
        targetPosition += degrees * POSITION_PER_DEGREE;
        targetPosition = Range.clip(targetPosition, MIN_POSITION, MAX_POSITION);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }

    private double moveTowards(double current, double target, double maxDelta) {
        if (target > current) {
            return Math.min(target, current + maxDelta);
        }
        return Math.max(target, current - maxDelta);
    }

    private void applyPosition(double position) {
        turel.setPosition(position);
        turel1.setPosition(MIRROR_SECOND_SERVO ? 1.0 - position : position);
    }
}
