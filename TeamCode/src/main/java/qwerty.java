package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "qwerty (Blocks to Java)")
public class qwerty extends LinearOpMode {
    private DcMotor ArtOn;
    private DcMotor ArtSolProb;
    private DcMotor AldOn;
    private DcMotor AldSol;

    private Servo bir;

    @Override
    public void runOpMode() {
        ArtOn = hardwareMap.get(DcMotor.class, "ArtOn");
        ArtSolProb = hardwareMap.get(DcMotor.class, "ArtSolProb");
        AldOn = hardwareMap.get(DcMotor.class, "AldOn");
        AldSol = hardwareMap.get(DcMotor.class, "AldSol");

        // Put initialization blocks here.
        waitForStart();
        AldOn.setDirection(DcMotor.Direction.FORWARD);
        AldSol.setDirection(DcMotor.Direction.REVERSE);
        ArtOn.setDirection(DcMotor.Direction.FORWARD);
        ArtSolProb.setDirection(DcMotor.Direction.REVERSE);
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                Juru();
                telemetry.update();
            }
        }
    }
    private void Juru(){
        if (gamepad1.right_bumper) {
            ((DcMotorEx) ArtOn).setVelocity(1125);
            ((DcMotorEx) ArtSolProb).setVelocity(3000);
            ((DcMotorEx) AldOn).setVelocity(1125);
            ((DcMotorEx) AldSol).setVelocity(1125);
        }else if (gamepad1.left_bumper) {
            ((DcMotorEx) ArtOn).setVelocity(-1125);
            ((DcMotorEx) ArtSolProb).setVelocity(-3000);
            ((DcMotorEx) AldOn).setVelocity(-1125);
            ((DcMotorEx) AldSol).setVelocity(-1125);
        }else if (gamepad1.dpad_left) {
            ((DcMotorEx) ArtOn).setVelocity(1125);
            ((DcMotorEx) ArtSolProb).setVelocity(-3000);
            ((DcMotorEx) AldOn).setVelocity(1125);
            ((DcMotorEx) AldSol).setVelocity(-1125);

        }else if (gamepad1.dpad_right) {
            ((DcMotorEx) ArtOn).setVelocity(-1125);
            ((DcMotorEx) ArtSolProb).setVelocity(3000);
            ((DcMotorEx) AldOn).setVelocity(-1125);
            ((DcMotorEx) AldSol).setVelocity(1125);

        }else{
            ((DcMotorEx) ArtOn).setVelocity(0);
            ((DcMotorEx) ArtSolProb).setVelocity(0);
            ((DcMotorEx) AldOn).setVelocity(0);
            ((DcMotorEx) AldSol).setVelocity(0);
        }
    }
}
