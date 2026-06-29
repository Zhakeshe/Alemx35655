package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controllers.Action;
import org.firstinspires.ftc.teamcode.Controllers.Base;
import org.firstinspires.ftc.teamcode.Controllers.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "AlemOp - Red", group = "TeleOp")
public class AlemOp extends LinearOpMode {

    private Base robotBase;
    private Action action;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);

        Pose startPose = new Pose(72, 72, Math.toRadians(0));
        follower.setStartingPose(startPose);
        follower.setPose(startPose);

        robotBase = new Base();
        action = new Action(hardwareMap);

        robotBase.initialize(hardwareMap, true);
        Action.isBlue = false;

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robotBase.update(x, y, rx, 1.0, false, false);

            if (gamepad1.options) {
                robotBase.resetHeading(0);
            }

            if (gamepad1.right_trigger > 0.1) {
                action.take();
            } else if (gamepad1.left_trigger > 0.1) {
                action.Sequence();
            } else {
                action.stopTake();
            }

            if (gamepad1.right_bumper) {
                action.runCloseShooter();
            } else if (gamepad1.left_bumper) {
                action.stopShooter();
            }

            if (gamepad1.triangle) {
                action.runFarShoot();
                action.servoUp();
            }
            if (gamepad1.a){
                action.servoMidle();
            }

            if (gamepad1.square) {
                action.setTurretMode(Turret.Mode.ODOMETRY);
            } else if (gamepad1.dpad_left) {
                action.setTurretMode(Turret.Mode.MANUAL);
            }

            if (gamepad1.b) {
                Pose resetP = new Pose(125.3, 79.2, 0);
                follower.setPose(resetP);
                robotBase.resetHeading(0);
            }

            action.updateAll(currentPose);

            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Turret Angle", action.getTurretAngle());
            telemetry.addData("Shooter RPM L", action.getShooterCurrentRPML());
            telemetry.addData("Shooter RPM R", action.getShooterCurrentRPMR());
            telemetry.update();
        }
    }
}