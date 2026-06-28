package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "AUTO Red2 zhorik", group = "Digital")
public class Autored2 extends OpMode {

    // ── Шутер ──────────────────────────────────────────────────────────────────
    public static double SHOOTER_VELO_MAX  = 1640.0;
    public static double SHOOTER_VELO_IDLE = 900.0;

    // ── Интейк / трансфер ──────────────────────────────────────────────────────
    public static double INTAKE_POWER_COLLECT = 1.0;
    public static double INTAKE_POWER_HOLD    = 1.0;
    public static double TRANSFER_POWER_HOLD  = 1.0;
    public static double INTAKE_POWER_SHOOT   = 1.0;
    public static double TRANSFER_POWER_SHOOT = 1.0;

    // ── Стоппер ────────────────────────────────────────────────────────────────
    public static double STOPPER_OPEN   = 0.08;
    public static double STOPPER_CLOSED = 0.48;

    // ── Шутер тұрақтылығы ─────────────────────────────────────────────────────
    public static double SHOOTER_READY_TOLERANCE = 30.0;
    public static double SHOOTER_STABLE_WINDOW   = 0.10;

    // ── Ату уақыттары ─────────────────────────────────────────────────────────
    public static double FIRST_SHOT_WARMUP_DELAY    = 0.8;
    public static double FOLLOWUP_SHOT_WARMUP_DELAY = 0.12;
    public static double SHOT_FORCE_OPEN_DELAY      = 0.60;
    public static double TIME_TO_SHOOT              = 0.50;

    // ── Турель позициялары (2 серво бірге) ────────────────────────────────────
    public static double TUREL_SHOT_1_POSITION = 0.78;
    public static double TUREL_SHOT_2_POSITION = 0.73;
    public static double TUREL_PARK_POSITION   = 0.55;

    // ── Жинау күту ────────────────────────────────────────────────────────────
    public static double TIME_TO_COLLECT_WAIT = 0.35;

    // ── Жалпы таймерлер ───────────────────────────────────────────────────────
    public static double FIRST_SHOOT_DELAY   = 2.2;
    public static double EMERGENCY_EXIT_TIME = 29.0;
    public static double MAX_PATH_TIMEOUT    = 6.0;

    // =========================================================================
    // Аппараттық нысандар
    // =========================================================================
    private Follower  follower;
    private DcMotorEx intake;
    private DcMotorEx shooter1;   // 1-ші шутер моторы
    private DcMotorEx shooter2;   // 2-ші шутер моторы
    private DcMotorEx transfer;
    private Servo     turel;      // 1-ші турель серво
    private Servo     turel1;     // 2-ші турель серво
    private Servo     stopper;
    private double    turelHoldPosition;
    private double    turel1HoldPosition;

    private final Timer shooterStableTimer   = new Timer();
    private boolean     shooterWasReady      = false;
    private double      waitCollectStartTime = -1.0;

    // =========================================================================
    // FSM
    // =========================================================================
    private enum ShootSubState { WARMUP, FIRING }
    private ShootSubState shootSubState = ShootSubState.WARMUP;

    private enum PathState {
        START_TO_SHOOT,   // Шут позициясына жету + RPM жинау
        SHOOT_1,          // 1-ші ату (шут позициясынан тікелей)
        GO_COLLECT_1,     // Collect1-ге контрол точка арқылы
        RETURN_1,         // Шут позициясына қайту
        SHOOT_2,          // 2-ші ату
        GO_COLLECT_2,     // Collect2-ге бару (1-ші рет)
        RETURN_2,         // Collect2-ден шутерге тікелей қайту
        SHOOT_3,          // 3-ші ату
        GO_COLLECT_3,     // Collect2-ге бару (2-ші рет, collect3 = collect2)
        RETURN_3,         // Collect2-ден шутерге тікелей қайту (2-ші рет)
        SHOOT_4,          // 4-ші ату
        PARK,             // Парковка
        END
    }

    private PathState   pathState     = PathState.START_TO_SHOOT;
    private final Timer stateTimer    = new Timer();
    private boolean     firstEnter    = true;
    private boolean     emergencyExit = false;

    // =========================================================================
    // Жолдар
    // =========================================================================
    private PathChain startToShoot;
    private PathChain goCollect1, return1;
    private PathChain goCollect2, return2;
    private PathChain goCollect3, return3;
    private PathChain park;

    // =========================================================================
    // Координаттар (Pedro Pathing Visualizer-ден алынған)
    // =========================================================================

    // Старт: 90 градус
    private final Pose startPose = new Pose(83.78237455435355, 16.5988619399169894, Math.toRadians(90));

    // Шут позициясы (Path 2-ден: Constant 90°)
    private final Pose shootPose = new Pose(83.78237455435355, 16.5988619399169894, Math.toRadians(90));

    // Collect1 (Path 1): контрол точка арқылы
    private final Pose collectControlPose1 = new Pose(87.6690, 38.5417, Math.toRadians(0));
    private final Pose collectPose1        = new Pose(127.5577533, 34.3638474, Math.toRadians(0));

    // Collect2 / Collect3 — бір ғана жинау позициясы (екеуі де осыны қолданады)
    private final Pose collectPose2 = new Pose(133, 12 , Math.toRadians(0));

    // Парковка (шут позициясына жақын)
    private final Pose parkPose = new Pose(84.05573904033487, 26.080170351131947, Math.toRadians(90));

    // =========================================================================
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        intake   = hardwareMap.get(DcMotorEx.class, "intake");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        turel    = hardwareMap.get(Servo.class, "turel");
        turel1   = hardwareMap.get(Servo.class, "turel1");
        stopper  = hardwareMap.get(Servo.class, "stopper");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setTurelShot1Position();
        holdTurelServos();
        buildPaths();
        stopAllMechanisms();
        closeStopper();
    }

    @Override
    public void start() {
        pathState            = PathState.START_TO_SHOOT;
        firstEnter           = true;
        emergencyExit        = false;
        shooterWasReady      = false;
        waitCollectStartTime = -1.0;

        stateTimer.resetTimer();
        closeStopper();
        setTurelShot1Position();
        holdTurelServos();

        // Бастаған кезде сразу MAX жылдамдық
        shooter1.setVelocity(SHOOTER_VELO_MAX);
        shooter2.setVelocity(SHOOTER_VELO_MAX);

        intake.setPower(INTAKE_POWER_HOLD);
        transfer.setPower(TRANSFER_POWER_HOLD);
    }

    @Override
    public void loop() {
        follower.update();
        holdTurelServos();

        if (pathState == PathState.END) {
            stopAllMechanisms();
            holdTurelServos();
            closeStopper();
            telemetry.addData("State", "END / IDLE");
            telemetry.update();
            return;
        }

        updateFSM();

        telemetry.addData("State",       pathState);
        telemetry.addData("SubState",    shootSubState);
        telemetry.addData("Shooter1",    "%.1f", shooter1.getVelocity());
        telemetry.addData("Shooter2",    "%.1f", shooter2.getVelocity());
        telemetry.addData("Diff1",       "%.1f", SHOOTER_VELO_MAX - shooter1.getVelocity());
        telemetry.addData("Diff2",       "%.1f", SHOOTER_VELO_MAX - shooter2.getVelocity());
        telemetry.addData("Ready",       shooterReady());
        telemetry.addData("StableFor",   "%.3f s", shooterWasReady ? shooterStableTimer.getElapsedTimeSeconds() : 0.0);
        telemetry.addData("StateTimer",  "%.2f s", stateTimer.getElapsedTimeSeconds());
        telemetry.addData("Emergency",   emergencyExit);
        telemetry.addData("Pose",        follower.getPose());
        telemetry.update();
    }

    // =========================================================================
    // FSM жаңарту
    // =========================================================================
    private void updateFSM() {
        // Апаттық шығу (29 секунд)
        if (!emergencyExit && getRuntime() > EMERGENCY_EXIT_TIME) {
            emergencyExit = true;
            pathState  = PathState.PARK;
            firstEnter = true;
        }

        // PARK/END-тен басқа жерде шутер MAX жылдамдықта + турель жаңарту
        if (pathState != PathState.PARK && pathState != PathState.END) {
            shooter1.setVelocity(SHOOTER_VELO_MAX);
            shooter2.setVelocity(SHOOTER_VELO_MAX);
            updateTurelForCurrentState();
        }

        switch (pathState) {

            // ── Шут позициясына жету + RPM жинау ────────────────────────────
            case START_TO_SHOOT:
                followOnce(startToShoot);
                keepIntakeRunning();
                if (pathTimedOut()) {
                    emergencyPark();
                } else if (stateTimer.getElapsedTimeSeconds() > FIRST_SHOOT_DELAY) {
                    next(PathState.SHOOT_1);
                }
                break;

            // ── 1-ші ату ────────────────────────────────────────────────────
            case SHOOT_1:
                shootFor(PathState.GO_COLLECT_1);
                break;

            // ── Collect1-ге контрол точка арқылы ────────────────────────────
            case GO_COLLECT_1:
                followOnce(goCollect1);
                collectIntake();
                advanceWhenPathDone(PathState.RETURN_1);
                break;

            case RETURN_1:
                followOnce(return1);
                keepIntakeRunning();
                advanceWhenPathDone(PathState.SHOOT_2);
                break;

            // ── 2-ші ату ────────────────────────────────────────────────────
            case SHOOT_2:
                shootFor(PathState.GO_COLLECT_2);
                break;

            // ── Collect2-ге 1-ші рет ────────────────────────────────────────
            case GO_COLLECT_2:
                followOnce(goCollect2);
                collectIntake();
                advanceWhenPathDone(PathState.RETURN_2);
                break;

            // ── Collect2-ден шутерге тікелей қайту ──────────────────────────
            case RETURN_2:
                followOnce(return2);
                keepIntakeRunning();
                advanceWhenPathDone(PathState.SHOOT_3);
                break;

            // ── 3-ші ату ────────────────────────────────────────────────────
            case SHOOT_3:
                shootFor(PathState.GO_COLLECT_3);
                break;

            // ── Collect2-ге 2-ші рет (collect3 = collect2 нүктесі) ──────────
            case GO_COLLECT_3:
                followOnce(goCollect3);
                collectIntake();
                advanceWhenPathDone(PathState.RETURN_3);
                break;

            // ── Collect2-ден шутерге тікелей қайту (2-ші рет) ───────────────
            case RETURN_3:
                followOnce(return3);
                keepIntakeRunning();
                advanceWhenPathDone(PathState.SHOOT_4);
                break;

            // ── 4-ші ату ────────────────────────────────────────────────────
            case SHOOT_4:
                shootFor(PathState.PARK);
                break;

            // ── Парковка ─────────────────────────────────────────────────────
            case PARK:
                followOnce(park);
                setTurelParkPosition();
                holdTurelServos();
                shooter1.setVelocity(SHOOTER_VELO_IDLE);
                shooter2.setVelocity(SHOOTER_VELO_IDLE);
                intake.setPower(0);
                transfer.setPower(0);
                closeStopper();
                if (parkDone()) next(PathState.END);
                break;

            case END:
                stopAllMechanisms();
                setTurelParkPosition();
                holdTurelServos();
                closeStopper();
                break;
        }
    }

    // =========================================================================
    // Ату логикасы
    // =========================================================================
    private void shootFor(PathState nextState) {
        if (firstEnter) {
            closeStopper();
            shootSubState   = ShootSubState.WARMUP;
            shooterWasReady = false;
            stateTimer.resetTimer();
            firstEnter = false;
        }

        switch (shootSubState) {
            case WARMUP:
                closeStopper();
                keepIntakeRunning();
                if (canOpenStopper(stateTimer.getElapsedTimeSeconds())) {
                    openStopper();
                    intake.setPower(INTAKE_POWER_SHOOT);
                    transfer.setPower(TRANSFER_POWER_SHOOT);
                    shootSubState = ShootSubState.FIRING;
                    stateTimer.resetTimer();
                }
                break;

            case FIRING:
                // Ату кезінде де MAX жылдамдық
                shooter1.setVelocity(SHOOTER_VELO_MAX);
                shooter2.setVelocity(SHOOTER_VELO_MAX);
                intake.setPower(INTAKE_POWER_SHOOT);
                transfer.setPower(TRANSFER_POWER_SHOOT);
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    closeStopper();
                    next(nextState);
                }
                break;
        }
    }

    private boolean canOpenStopper(double elapsedSeconds) {
        if (elapsedSeconds <= currentShootWarmupDelay()) {
            shooterWasReady = false;
            return false;
        }
        // Мәжбүрлеп ашу (уақыт өтсе)
        if (elapsedSeconds > SHOT_FORCE_OPEN_DELAY) {
            return true;
        }
        // Шутер дайын болса + тұрақты болса
        if (shooterReady()) {
            if (!shooterWasReady) {
                shooterStableTimer.resetTimer();
                shooterWasReady = true;
            }
            return shooterStableTimer.getElapsedTimeSeconds() >= SHOOTER_STABLE_WINDOW;
        } else {
            shooterWasReady = false;
            return false;
        }
    }

    private boolean shooterReady() {
        double v1 = shooter1.getVelocity();
        double v2 = shooter2.getVelocity();
        return v1 > 0 && v2 > 0
                && Math.abs(v1 - SHOOTER_VELO_MAX) <= SHOOTER_READY_TOLERANCE
                && Math.abs(v2 - SHOOTER_VELO_MAX) <= SHOOTER_READY_TOLERANCE;
    }

    private double currentShootWarmupDelay() {
        return pathState == PathState.SHOOT_1 ? FIRST_SHOT_WARMUP_DELAY : FOLLOWUP_SHOT_WARMUP_DELAY;
    }

    // =========================================================================
    // Жол утилиттері
    // =========================================================================
    private void followOnce(PathChain path) {
        if (firstEnter) {
            follower.followPath(path, true);
            stateTimer.resetTimer();
            firstEnter = false;
        }
    }

    private boolean pathTimedOut() {
        return follower.isBusy() && stateTimer.getElapsedTimeSeconds() > MAX_PATH_TIMEOUT;
    }

    private boolean parkDone() {
        return !follower.isBusy() || stateTimer.getElapsedTimeSeconds() > MAX_PATH_TIMEOUT;
    }

    private boolean advanceWhenPathDone(PathState nextState) {
        if (!follower.isBusy()) {
            next(nextState);
            return true;
        }
        if (pathTimedOut()) {
            emergencyPark();
            return true;
        }
        return false;
    }

    private void emergencyPark() {
        intake.setPower(0);
        transfer.setPower(0);
        emergencyExit = true;
        pathState  = PathState.PARK;
        firstEnter = true;
    }

    private void next(PathState nextState) {
        pathState  = nextState;
        firstEnter = true;
    }

    // =========================================================================
    // Интейк / трансфер
    // =========================================================================
    private void collectIntake() {
        intake.setPower(INTAKE_POWER_COLLECT);
        transfer.setPower(INTAKE_POWER_COLLECT);
        closeStopper();
    }

    private void keepIntakeRunning() {
        intake.setPower(INTAKE_POWER_HOLD);
        transfer.setPower(TRANSFER_POWER_HOLD);
        closeStopper();
    }

    // =========================================================================
    // Серво утилиттері
    // =========================================================================
    private void openStopper()  { stopper.setPosition(STOPPER_OPEN);   }
    private void closeStopper() { stopper.setPosition(STOPPER_CLOSED); }

    private void holdTurelServos() {
        turel.setPosition(turelHoldPosition);
        turel1.setPosition(turel1HoldPosition);
    }

    private void setTurelShot1Position() {
        turelHoldPosition  = TUREL_SHOT_1_POSITION;
        turel1HoldPosition = TUREL_SHOT_1_POSITION;
    }

    private void setTurelShot2Position() {
        turelHoldPosition  = TUREL_SHOT_2_POSITION;
        turel1HoldPosition = TUREL_SHOT_2_POSITION;
    }

    private void setTurelParkPosition() {
        turelHoldPosition  = TUREL_PARK_POSITION;
        turel1HoldPosition = TUREL_PARK_POSITION;
    }

    // SHOOT_1-де TUREL_SHOT_1, қалған аттарда TUREL_SHOT_2
    private void updateTurelForCurrentState() {
        if (pathState == PathState.START_TO_SHOOT || pathState == PathState.SHOOT_1) {
            setTurelShot1Position();
        } else {
            setTurelShot2Position();
        }
    }

    private void stopAllMechanisms() {
        intake.setPower(0);
        transfer.setPower(0);
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
    }

    // =========================================================================
    // Жолдарды құру (buildPaths)
    // =========================================================================
    private void buildPaths() {
        // Старттан шут позициясына (90° → 90°)
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // ── Collect1: контрол точка арқылы Безье қисығы ──────────────────────
        goCollect1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, collectControlPose1, collectPose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collectPose1.getHeading())
                .build();

        // Collect1-ден шут позициясына тікелей қайту
        return1 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose1, shootPose))
                .setLinearHeadingInterpolation(collectPose1.getHeading(), shootPose.getHeading())
                .build();

        // ── Shooter-ден Collect2-ге тікелей бару (1-ші рет) ─────────────────
        goCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collectPose2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collectPose2.getHeading())
                .build();

        // Collect2-ден шутерге тікелей қайту
        return2 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose2, shootPose))
                .setLinearHeadingInterpolation(collectPose2.getHeading(), shootPose.getHeading())
                .build();

        // ── Shooter-ден Collect2-ге тікелей бару (2-ші рет) ─────────────────
        goCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collectPose2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collectPose2.getHeading())
                .build();

        // Collect2-ден шутерге тікелей қайту (2-ші рет)
        return3 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose2, shootPose))
                .setLinearHeadingInterpolation(collectPose2.getHeading(), shootPose.getHeading())
                .build();

        // ── Парковка ─────────────────────────────────────────────────────────
        park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }
}