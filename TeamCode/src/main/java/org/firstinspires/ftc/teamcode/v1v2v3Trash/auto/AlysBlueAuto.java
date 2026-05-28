package org.firstinspires.ftc.teamcode.v1v2v3Trash.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AlysBlueAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    autoShoot shoot = new autoShoot();

    private boolean shotsTriger = false;
    private boolean intakeTriger = false;

    private Servo xl, xr;

    public enum PathState{
        Start_ShootPos,
        ShootPos_PreDop,
        PreDop_Dop,
        Dop_ShootPos,
        ShootPos_Park
    }


    private final Pose startPose = new Pose(55.47636363636364, 8.349090909090911, Math.toRadians(90));
    private final Pose shootPos = new Pose(69, 23, Math.toRadians(116));
    private final Pose preDop = new Pose(23.359999999999996, 8.349090909090911, Math.toRadians(180));
    private final Pose dop = new Pose(8, 8.349090909090911, Math.toRadians(180));
    private final Pose park = new Pose(38, 21, Math.toRadians(160));

    private PathChain start2shoot, shoot2Predop, preDop2dop, dop2shoot, shoot2Park;
    public void buildPath(){
        start2shoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPos))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPos.getHeading())
                .build();
        shoot2Predop = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, preDop))
                .setLinearHeadingInterpolation(shootPos.getHeading(), preDop.getHeading())
                .build();
        preDop2dop = follower.pathBuilder()
                .addPath(new BezierLine(preDop, dop))
                .setLinearHeadingInterpolation(preDop.getHeading(), dop.getHeading())
                .build();
        dop2shoot = follower.pathBuilder()
                .addPath(new BezierLine(dop, startPose))
                .setLinearHeadingInterpolation(dop.getHeading(), startPose.getHeading())
                .build();
        shoot2Park = follower.pathBuilder()
                .addPath(new BezierLine(startPose, park))
                .setLinearHeadingInterpolation(startPose.getHeading(), park.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case Start_ShootPos:
                follower.followPath(start2shoot, true);
                setPathState(PathState.ShootPos_PreDop);
                break;
            case ShootPos_PreDop:
                if (!shotsTriger){
                    shoot.HighShoots(1);
                    shotsTriger = true;
                } else if (shotsTriger && shoot.highisbusy()) {
                    shotsTriger = false;
                    follower.followPath(shoot2Predop, true);
                    setPathState(PathState.PreDop_Dop);
                }
                break;
            case PreDop_Dop:
                if (!follower.isBusy()){
                    if (!intakeTriger){
                        shoot.InShots(1);
                        intakeTriger = true;
                    } else if (intakeTriger) {
                        intakeTriger = false;
                        follower.followPath(preDop2dop, true);
                        setPathState(PathState.Dop_ShootPos);
                    }
                }
                break;
            case Dop_ShootPos:
                if (!follower.isBusy()){
                    follower.followPath(dop2shoot, true);
                    setPathState(PathState.ShootPos_Park);
                }
                break;
            case ShootPos_Park:
                if (!follower.isBusy()){
                    if (!shotsTriger){
                        shoot.HighShoots(1);
                        shotsTriger = true;
                    } else if (shotsTriger && shoot.highisbusy()) {
                        shotsTriger = false;
                        follower.followPath(shoot2Park, true);
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
        pathState = PathState.Start_ShootPos;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //
        shoot.init(hardwareMap);

        xl = hardwareMap.get(Servo.class, "xl");
        xr = hardwareMap.get(Servo.class, "xr");
        xl.setPosition(0.5);
        xr.setPosition(0.5);


        buildPath();
        follower.setPose(startPose);
    }
    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        shoot.highShooter();
        shoot.intake();
        statePathUpdate();
    }
}
