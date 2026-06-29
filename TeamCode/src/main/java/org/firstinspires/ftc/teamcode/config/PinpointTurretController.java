package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public class PinpointTurretController {

    public static double START_X = 108.0;
    public static double START_Y = 58.0;
    public static double START_HEADING_DEG = 0.0;

    public static double CENTER_POSITION = 0.6267;
    public static double MIN_POSITION = 0.00;
    public static double MAX_POSITION = 1.00;
    public static double POSITION_PER_DEGREE = 1.0 / 300.0;
    public static double CENTER_RELATIVE_ANGLE_DEG = 0.0;
    public static double TURRET_DIRECTION = -1.0;
    public static double COARSE_FIELD_STEP_DEG = 10.0;
    public static double FINE_FIELD_STEP_DEG = 2.0;
    public static double FIELD_LOCK_GAIN = 1.0;
    public static double TURN_RATE_COMPENSATION_GAIN = 1.46;
    public static double TURN_INPUT_COMPENSATION_GAIN = 1.18;
    public static double HEADING_LOOKAHEAD_SEC = 0.28;
    public static double TURN_INPUT_FEEDFORWARD_DEG = 18.0;
    public static double LARGE_ERROR_START_DEG = 12.0;
    public static double LARGE_ERROR_GAIN = 0.55;
    public static double TURN_INPUT_DEADBAND = 0.05;
    public static double TURN_RATE_SETTLED_DEG_PER_SEC = 8.0;
    public static double AIM_DEADBAND_DEG = 0.02;
    public static boolean MIRROR_SECOND_SERVO = false;

    private GoBildaPinpointDriver pinpointDriver;
    private Servo turel;
    private Servo turel1;

    private double lockedFieldHeadingDeg = START_HEADING_DEG;
    private double targetPosition = CENTER_POSITION;
    private double lastLoopTime;
    private double lastHeadingDeg = START_HEADING_DEG;
    private double continuousHeadingDeg = START_HEADING_DEG;
    private double turnRateDegPerSec;

    private boolean dpadLeftWasPressed;
    private boolean dpadRightWasPressed;
    private boolean dpadUpWasPressed;
    private boolean dpadDownWasPressed;

    public void init(HardwareMap hardwareMap) {
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PINPOINT_HARDWARE_NAME);
        pinpointDriver.setOffsets(-15, -7.5, DistanceUnit.CM);
        pinpointDriver.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpointDriver.update();

        turel = hardwareMap.get(Servo.class, "turel");
        turel1 = hardwareMap.get(Servo.class, "turel1");

        targetPosition = Range.clip(CENTER_POSITION, MIN_POSITION, MAX_POSITION);
        double currentHeadingDeg = getCurrentHeadingDeg();
        lockedFieldHeadingDeg = getCurrentFieldHeadingDeg(currentHeadingDeg, targetPosition);
        applyPosition(targetPosition);
        lastLoopTime = 0.0;
        lastHeadingDeg = currentHeadingDeg;
        continuousHeadingDeg = currentHeadingDeg;
        turnRateDegPerSec = 0.0;
    }

    public void update(Gamepad gamepad, Telemetry telemetry, double runtimeSeconds) {
        if (pinpointDriver == null) {
            return;
        }

        double deltaTime = Math.max(0.0, runtimeSeconds - lastLoopTime);
        lastLoopTime = runtimeSeconds;

        pinpointDriver.update();
        updateHeadingState(deltaTime);

        handleButtons(gamepad);
        targetPosition = computeFieldLockPosition(gamepad);
        applyPosition(targetPosition);

        telemetry.addData("Turret Lock Heading", lockedFieldHeadingDeg);
        telemetry.addData("Turret Position", targetPosition);
        telemetry.addData("Turret Relative Deg", getRelativeTurretDegrees());
        telemetry.addData("Turret Turn Rate", turnRateDegPerSec);
    }

    private void handleButtons(Gamepad gamepad) {
        boolean dpadLeftPressed = gamepad.dpad_left;
        boolean dpadRightPressed = gamepad.dpad_right;
        boolean dpadUpPressed = gamepad.dpad_up;
        boolean dpadDownPressed = gamepad.dpad_down;

        if (dpadDownPressed && !dpadDownWasPressed) {
            lockedFieldHeadingDeg -= FINE_FIELD_STEP_DEG;
        }

        if (dpadUpPressed && !dpadUpWasPressed) {
            lockedFieldHeadingDeg += FINE_FIELD_STEP_DEG;
        }

        if (dpadLeftPressed && !dpadLeftWasPressed) {
            lockedFieldHeadingDeg -= COARSE_FIELD_STEP_DEG;
        }

        if (dpadRightPressed && !dpadRightWasPressed) {
            lockedFieldHeadingDeg += COARSE_FIELD_STEP_DEG;
        }

        dpadLeftWasPressed = dpadLeftPressed;
        dpadRightWasPressed = dpadRightPressed;
        dpadUpWasPressed = dpadUpPressed;
        dpadDownWasPressed = dpadDownPressed;
    }

    private double computeFieldLockPosition(Gamepad gamepad) {
        double baseRelativeDegrees =
                normalizeDegrees(lockedFieldHeadingDeg - continuousHeadingDeg) * FIELD_LOCK_GAIN;
        double boostedRelativeDegrees = applyLargeErrorBoost(baseRelativeDegrees);
        double turnInput = getTurnInput(gamepad);
        if (Math.abs(turnInput) < TURN_INPUT_DEADBAND) {
            turnInput = 0.0;
        }
        double effectiveTurnRateDegPerSec = turnRateDegPerSec;
        if (turnInput == 0.0 && Math.abs(effectiveTurnRateDegPerSec) < TURN_RATE_SETTLED_DEG_PER_SEC) {
            effectiveTurnRateDegPerSec = 0.0;
        }
        double turnRateCompensationDeg =
                effectiveTurnRateDegPerSec * HEADING_LOOKAHEAD_SEC * TURN_RATE_COMPENSATION_GAIN;
        double turnInputCompensationDeg =
                turnInput * TURN_INPUT_FEEDFORWARD_DEG * TURN_INPUT_COMPENSATION_GAIN;
        double relativeDegrees =
                clampRelativeDegrees(boostedRelativeDegrees - turnRateCompensationDeg + turnInputCompensationDeg);
        if (Math.abs(baseRelativeDegrees) < AIM_DEADBAND_DEG
                && Math.abs(turnRateCompensationDeg) < AIM_DEADBAND_DEG
                && Math.abs(turnInputCompensationDeg) < AIM_DEADBAND_DEG) {
            relativeDegrees = 0.0;
        }
        return relativeDegreesToPosition(relativeDegrees);
    }

    private double getRelativeTurretDegrees() {
        return clampRelativeDegrees(normalizeDegrees(lockedFieldHeadingDeg - continuousHeadingDeg));
    }

    private double getCurrentFieldHeadingDeg(double currentHeadingDeg, double servoPosition) {
        return normalizeDegrees(currentHeadingDeg + positionToRelativeDegrees(servoPosition));
    }

    private double positionToRelativeDegrees(double position) {
        return (((position - CENTER_POSITION) / POSITION_PER_DEGREE) * TURRET_DIRECTION)
                + CENTER_RELATIVE_ANGLE_DEG;
    }

    private double relativeDegreesToPosition(double relativeDegrees) {
        double rawPosition = CENTER_POSITION
                + ((relativeDegrees - CENTER_RELATIVE_ANGLE_DEG) * POSITION_PER_DEGREE * TURRET_DIRECTION);
        return Range.clip(rawPosition, MIN_POSITION, MAX_POSITION);
    }

    private void updateHeadingState(double deltaTime) {
        double currentHeadingDeg = getCurrentHeadingDeg();
        double deltaHeadingDeg = normalizeDegrees(currentHeadingDeg - lastHeadingDeg);
        continuousHeadingDeg += deltaHeadingDeg;
        lastHeadingDeg = currentHeadingDeg;
        if (deltaTime <= 0.0) {
            return;
        }
        double measuredTurnRate = deltaHeadingDeg / deltaTime;
        turnRateDegPerSec = 0.20 * turnRateDegPerSec + 0.80 * measuredTurnRate;
    }

    private double applyLargeErrorBoost(double relativeDegrees) {
        double absRelativeDegrees = Math.abs(relativeDegrees);
        if (absRelativeDegrees <= LARGE_ERROR_START_DEG) {
            return relativeDegrees;
        }
        double boostedMagnitude = absRelativeDegrees
                + ((absRelativeDegrees - LARGE_ERROR_START_DEG) * LARGE_ERROR_GAIN);
        return Math.signum(relativeDegrees) * boostedMagnitude;
    }

    private double clampRelativeDegrees(double relativeDegrees) {
        double minRelativeDegrees = Math.min(
                positionToRelativeDegrees(MIN_POSITION),
                positionToRelativeDegrees(MAX_POSITION)
        );
        double maxRelativeDegrees = Math.max(
                positionToRelativeDegrees(MIN_POSITION),
                positionToRelativeDegrees(MAX_POSITION)
        );
        return Range.clip(relativeDegrees, minRelativeDegrees, maxRelativeDegrees);
    }

    private double getCurrentHeadingDeg() {
        Pose2D position = pinpointDriver.getPosition();
        return position.getHeading(AngleUnit.DEGREES);
    }

    private double getTurnInput(Gamepad gamepad) {
        return gamepad.left_stick_x;
    }

    private double normalizeDegrees(double angleDeg) {
        while (angleDeg > 180.0) {
            angleDeg -= 360.0;
        }
        while (angleDeg < -180.0) {
            angleDeg += 360.0;
        }
        return angleDeg;
    }

    private void applyPosition(double position) {
        turel.setPosition(position);
        turel1.setPosition(MIRROR_SECOND_SERVO ? 1.0 - position : position);
    }
}
