package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Pinpoint Turret TeleOp", group = "Digital")
public class PinpointTurretTeleOp extends OpMode {

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

    private Follower follower;
    private Servo turel;
    private Servo turel1;

    private double lockedFieldHeadingDeg = START_HEADING_DEG;
    private double targetPosition = CENTER_POSITION;
    private double appliedPosition = CENTER_POSITION;
    private double lastLoopTime;
    private double lastHeadingDeg = START_HEADING_DEG;
    private double continuousHeadingDeg = START_HEADING_DEG;
    private double turnRateDegPerSec;

    private boolean aWasPressed;
    private boolean yWasPressed;
    private boolean dpadLeftWasPressed;
    private boolean dpadRightWasPressed;
    private boolean dpadUpWasPressed;
    private boolean dpadDownWasPressed;
    private boolean leftBumperWasPressed;
    private boolean rightBumperWasPressed;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        turel = hardwareMap.get(Servo.class, "turel");
        turel1 = hardwareMap.get(Servo.class, "turel1");
        targetPosition = Range.clip(CENTER_POSITION, MIN_POSITION, MAX_POSITION);
        appliedPosition = targetPosition;
        lockedFieldHeadingDeg = getCurrentFieldHeadingDeg(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)), targetPosition);
        applyPosition(appliedPosition);
        lastLoopTime = getRuntime();
        lastHeadingDeg = START_HEADING_DEG;
        continuousHeadingDeg = START_HEADING_DEG;
        turnRateDegPerSec = 0.0;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double deltaTime = Math.max(0.0, now - lastLoopTime);
        lastLoopTime = now;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();
        updateHeadingState(deltaTime);

        handleButtons();
        targetPosition = computeFieldLockPosition();

        appliedPosition = targetPosition;
        applyPosition(appliedPosition);

        Pose pose = follower.getPose();
        telemetry.addData("Field Lock", true);
        telemetry.addData("Robot X", pose.getX());
        telemetry.addData("Robot Y", pose.getY());
        telemetry.addData("Robot Heading Deg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Locked Field Heading Deg", lockedFieldHeadingDeg);
        telemetry.addData("Turret Target Pos", targetPosition);
        telemetry.addData("Turret Applied Pos", appliedPosition);
        telemetry.addData("Relative Turret Deg", getRelativeTurretDegrees(pose));
        telemetry.addData("Field Lock Gain", FIELD_LOCK_GAIN);
        telemetry.addData("Turn Rate Comp Gain", TURN_RATE_COMPENSATION_GAIN);
        telemetry.addData("Turn Input Comp Gain", TURN_INPUT_COMPENSATION_GAIN);
        telemetry.addData("Large Error Start Deg", LARGE_ERROR_START_DEG);
        telemetry.addData("Large Error Gain", LARGE_ERROR_GAIN);
        telemetry.addData("Turn Input Deadband", TURN_INPUT_DEADBAND);
        telemetry.addData("Turn Settled Deg/S", TURN_RATE_SETTLED_DEG_PER_SEC);
        telemetry.addData("Turn Rate Deg/S", turnRateDegPerSec);
        telemetry.addData("Aim Deadband Deg", AIM_DEADBAND_DEG);
        telemetry.addData("Turret Direction", TURRET_DIRECTION);
        telemetry.addLine("Field lock always ON");
        telemetry.addLine("A = face front, Y = reset pose");
        telemetry.addLine("Bumpers = coarse turn, dpad up/down = fine turn");
        telemetry.update();
    }

    private void handleButtons() {
        boolean aPressed = gamepad1.a;
        boolean yPressed = gamepad1.y;
        boolean dpadLeftPressed = gamepad1.dpad_left;
        boolean dpadRightPressed = gamepad1.dpad_right;
        boolean dpadUpPressed = gamepad1.dpad_up;
        boolean dpadDownPressed = gamepad1.dpad_down;
        boolean leftBumperPressed = gamepad1.left_bumper;
        boolean rightBumperPressed = gamepad1.right_bumper;

        if (aPressed && !aWasPressed) {
            lockedFieldHeadingDeg = continuousHeadingDeg + CENTER_RELATIVE_ANGLE_DEG;
            targetPosition = relativeDegreesToPosition(CENTER_RELATIVE_ANGLE_DEG);
        }

        if (yPressed && !yWasPressed) {
            follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        }

        if (leftBumperPressed && !leftBumperWasPressed) {
            lockedFieldHeadingDeg -= COARSE_FIELD_STEP_DEG;
        }

        if (rightBumperPressed && !rightBumperWasPressed) {
            lockedFieldHeadingDeg += COARSE_FIELD_STEP_DEG;
        }

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

        aWasPressed = aPressed;
        yWasPressed = yPressed;
        dpadLeftWasPressed = dpadLeftPressed;
        dpadRightWasPressed = dpadRightPressed;
        dpadUpWasPressed = dpadUpPressed;
        dpadDownWasPressed = dpadDownPressed;
        leftBumperWasPressed = leftBumperPressed;
        rightBumperWasPressed = rightBumperPressed;
    }

    private double computeFieldLockPosition() {
        double baseRelativeDegrees =
                normalizeDegrees(lockedFieldHeadingDeg - continuousHeadingDeg) * FIELD_LOCK_GAIN;
        double boostedRelativeDegrees = applyLargeErrorBoost(baseRelativeDegrees);
        double turnInput = getTurnInput();
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

    private double getRelativeTurretDegrees(Pose pose) {
        return clampRelativeDegrees(normalizeDegrees(lockedFieldHeadingDeg - continuousHeadingDeg));
    }

    private double getCurrentFieldHeadingDeg(Pose pose, double servoPosition) {
        return normalizeDegrees(Math.toDegrees(pose.getHeading()) + positionToRelativeDegrees(servoPosition));
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

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    private void updateHeadingState(double deltaTime) {
        double currentHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
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

    private double getTurnInput() {
        return gamepad1.right_stick_x;
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
