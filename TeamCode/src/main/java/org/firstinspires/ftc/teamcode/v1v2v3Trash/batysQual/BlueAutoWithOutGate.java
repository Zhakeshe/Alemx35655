package org.firstinspires.ftc.teamcode.v1v2v3Trash.batysQual;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.FAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
public class BlueAutoWithOutGate extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private FAuto shooter = new FAuto();

    private boolean shotsTriger = false;
    private boolean intakeTriger = false;
    private boolean intakeTriger1 = false;

    private Servo z0;
    private Servo z1;


    public enum PathState{
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        Shoot2Dop1,
        Dop1TakeingDop,
        TakingDopShoot,

        Going2ShootPos,
        ShootDop2AndTakeDop,

        Dop2TakingDop,

        OpenGate2Shoot,
        ShootDop3,
    }

    private final Pose startPose = new Pose(27.406698564593288, 132.0191387559809, Math.toRadians(143));
    private final Pose shootPose = new Pose(35, 108, Math.toRadians(135));

    private final Pose take1Dop = new Pose(59.25358851674641, 57.5311004784689, Math.toRadians(180));
    private final Pose taking1Dop = new Pose(19.392344497607656, 57.186602870813395, Math.toRadians(180));

    private final Pose goingshootpose = new Pose(55, 79, Math.toRadians(135));

    private final Pose take2Dop = new Pose(45.468899521531114, 83.64593301435406, Math.toRadians(179));
    private final Pose taking2Dop = new Pose(20.334928229665074, 83.10047846889954, Math.toRadians(179));


    private final Pose Park = new Pose(16.928229665071772, 99.66028708133969, Math.toRadians(90));

    private PathChain driveStartPos2ShootPos, shootPos2take1Dop, take1Dop2TakingDop1, going2Shootpos,TakingDop12ShootPos, Shoot2takePos, takepos2takingDop, OpeningGate2ShootPos, ShotPos2Park;

    public void buildPaths(){
        driveStartPos2ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootPos2take1Dop = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, take1Dop))
                .setLinearHeadingInterpolation(shootPose.getHeading(), take1Dop.getHeading())
                .build();

        take1Dop2TakingDop1 = follower.pathBuilder()
                .addPath(new BezierLine(take1Dop, taking1Dop))
                .setLinearHeadingInterpolation(take1Dop.getHeading(), taking1Dop.getHeading())
                .build();

        going2Shootpos = follower.pathBuilder()
                .addPath(new BezierLine(taking1Dop, goingshootpose))
                .setLinearHeadingInterpolation(take1Dop.getHeading(), goingshootpose.getHeading())
                .build();

        TakingDop12ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(taking1Dop, shootPose))
                .setLinearHeadingInterpolation(taking1Dop.getHeading(), shootPose.getHeading())
                .build();

        Shoot2takePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, take2Dop))
                .setLinearHeadingInterpolation(shootPose.getHeading(), take2Dop.getHeading())
                .build();

        takepos2takingDop = follower.pathBuilder()
                .addPath(new BezierLine(take2Dop, taking2Dop))
                .setLinearHeadingInterpolation(take2Dop.getHeading(), taking2Dop.getHeading())
                .build();

        OpeningGate2ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(taking2Dop, shootPose))
                .setLinearHeadingInterpolation(taking2Dop.getHeading(), shootPose.getHeading())
                .build();

        ShotPos2Park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, Park))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Park.getHeading())
                .build();



    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                if (!intakeTriger1){
                    shooter.InShots1(1);
                    intakeTriger1 = true;
                }else if (intakeTriger1 && !shooter.isBusy()){
                    intakeTriger1 = false;
                    follower.followPath(driveStartPos2ShootPos, true);
                    setPathState(PathState.Shoot2Dop1);
                }
                break;

            case Shoot2Dop1:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shooter.FireShots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && !shooter.isBusy()) {
                        shotsTriger = false;
                        follower.followPath(shootPos2take1Dop, true);
                        setPathState(PathState.Dop1TakeingDop);
                    }
                }
                break;

            case Dop1TakeingDop:
                if (!follower.isBusy()){
                    if(!intakeTriger){
                        shooter.InShots(1);
                        intakeTriger = true;
                    } else if (intakeTriger && !shooter.isBusy()) {
                        follower.setMaxPower(0.5);
                        intakeTriger = false;
                        follower.followPath(take1Dop2TakingDop1, true);
                        setPathState(PathState.Going2ShootPos);
                    }
                }
                break;

            case Going2ShootPos:
                if (!follower.isBusy()){
                    follower.setMaxPower(1.0);
                    follower.followPath(going2Shootpos, true);
                    setPathState(PathState.TakingDopShoot);
                }
            case TakingDopShoot:
                if (!follower.isBusy()){
                    follower.followPath(TakingDop12ShootPos, true);
                    setPathState(PathState.ShootDop2AndTakeDop);
                }
                break;

            case ShootDop2AndTakeDop:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shooter.FireShots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && !shooter.isBusy()) {
                        shotsTriger = false;
                        follower.followPath(Shoot2takePos, true);
                        setPathState(PathState.Dop2TakingDop);
                    }
                }
                break;

            case Dop2TakingDop:
                if (!follower.isBusy()){
                    if (!intakeTriger){
                        shooter.InShots(1);
                        intakeTriger = true;
                    } else if (intakeTriger && !shooter.isBusy()) {
                        follower.setMaxPower(0.7);
                        intakeTriger = false;
                        follower.followPath(takepos2takingDop, true);
                        setPathState(PathState.OpenGate2Shoot);
                    }
                }
                break;
            case OpenGate2Shoot:
                if (!follower.isBusy()){
                    follower.followPath(OpeningGate2ShootPos, true);
                    setPathState(PathState. ShootDop3);
                }
                break;

            case ShootDop3:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shooter.FireShots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && !shooter.isBusy()) {
                        shotsTriger = false;
                        follower.followPath(ShotPos2Park, true);
                    }
                }
                break;

            default:
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    PathState pathState;

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);

        z0 = hardwareMap.get(Servo.class, "z0");
        z1 = hardwareMap.get(Servo.class, "z1");
        z0.setPosition(0);
        z1.setPosition(1);

        buildPaths();
        follower.setPose(startPose);

    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        shooter.intake();
        shooter.intake1();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();

    }
}
