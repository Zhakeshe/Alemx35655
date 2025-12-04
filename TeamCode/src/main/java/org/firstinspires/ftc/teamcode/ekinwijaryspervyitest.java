package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class ekinwijaryspervyitest extends LinearOpMode{
    private DcMotor AldySol;
    private DcMotor AldyOn;

    private DcMotor ArtySol;

    private DcMotor ArtyOn;
    @Override
    public void runOpMode() {
        AldySol = hardwareMap.get(DcMotor.class, "AldySol");
        AldyOn = hardwareMap.get(DcMotor.class, "AldyOn");
        ArtySol = hardwareMap.get(DcMotor.class, "ArtySol");
        ArtyOn = hardwareMap.get(DcMotor.class, "ArtyOn");

        AldySol.setDirection(DcMotor.Direction.REVERSE);
        AldyOn.setDirection(DcMotor.Direction.FORWARD);
        ArtySol.setDirection(DcMotor.Direction.FORWARD);
        ArtyOn.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        if(opModeIsActive()){
            while(opModeIsActive()){
                Kozgalu();
                telemetry.update();


            }

        }
    }

    public void Kozgalu(){
        double qy = gamepad1.right_trigger;
        double yy = -gamepad1.left_trigger;
        double turn = gamepad1.left_stick_x;
        double x = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(qy) + Math.abs(yy) + Math.abs(x) + Math.abs(turn), 1);

        AldySol.setPower(( (qy + yy + x + turn) / denominator) / 2.666667);
        ArtySol.setPower(( (qy + yy - x + turn) / denominator)/ 2.666667);
        AldyOn.setPower( (qy + yy - x - turn) / denominator);
        ArtyOn.setPower(( (qy + yy + x - turn) / denominator) / 2.666667);
    }
}

