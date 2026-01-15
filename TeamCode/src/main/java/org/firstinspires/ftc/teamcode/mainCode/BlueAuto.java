package org.firstinspires.ftc.teamcode.mainCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueAuto extends OpMode {


     private Follower follower;
     private Timer pathTimer, opModeTimer;

     public enum PathState{
         DRIVE_STARTPOS_SHOOT_POS,
         SHOOT_PRELOAD,
         Shoot2Dop1
    }

     private final Pose startPose = new Pose(21.03349282296649, 122.88995215311009, Math.toRadians(136));
     private final Pose shootPose = new Pose(49.110047846889934, 94.29665071770339, Math.toRadians(136));

     private final Pose take1Dop = new Pose(43.483253588516746, 60.334928229665074, Math.toRadians(180));

     private PathChain driveStartPos2ShootPos, shootPos2take1Dop;

     public void buildPaths(){
         driveStartPos2ShootPos = follower.pathBuilder()
                 .addPath(new BezierLine(startPose, shootPose))
                 .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                 .build();

         shootPos2take1Dop = follower.pathBuilder()
                 .addPath(new BezierLine(shootPose, take1Dop))
                 .setLinearHeadingInterpolation(shootPose.getHeading(), take1Dop.getHeading())
                 .build();

     }

     public void statePathUpdate(){
         switch (pathState){
             case DRIVE_STARTPOS_SHOOT_POS:
                 follower.followPath(driveStartPos2ShootPos, true);
                 setPathState(PathState.SHOOT_PRELOAD);
                 break;
             case SHOOT_PRELOAD:
                 if (!follower.isBusy()){
                     //Shooter qosy kerek
                     setPathState(PathState.Shoot2Dop1);
                 }
                 break;
             case Shoot2Dop1:
                 if (!follower.isBusy()){
                     follower.followPath(shootPos2take1Dop, true);
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
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();

    }
}
