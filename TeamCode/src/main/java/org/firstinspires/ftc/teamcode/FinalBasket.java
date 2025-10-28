package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "FinalBasket", group = "examples")
public class FinalBasket extends LinearOpMode {
    private Servo IntakeKoteru1; //0Control
    private Servo IntakeKoteru2; //1Control
    private Servo Intake; //2Control
    private Servo IntakeKozgalu; //3Control


    //private DcMotor AldOn; //1Exp
    //private DcMotor AldSol; //0Exp
    //private DcMotor ArtOn; //3Exp
    //private DcMotor ArtSol; //2Exp
    private DcMotor leftFront; //0exp
    private DcMotor leftRear; //2exp
    private DcMotor rightFront; //1exp
    private DcMotor rightRear; //3exp


    private DcMotor LiftBir; //3Control
    private DcMotor LiftEki; //2Control

    private DcMotor Oku1; //0Control
    private DcMotor Oku2; //1Control


    private Servo OutKoteru1; //0exp
    private Servo OutKoteru2; //1exp
    private Servo OutKoteru3; //2exp
    private Servo OutKozgalu; //3exp
    private Servo Outtake; //4exp

    boolean qwe = false;
    boolean wqe = false;

    int TPos = 304;

    @Override
    public void runOpMode() {
        IntakeKoteru1 = hardwareMap.get(Servo.class, "IntakeKoteru1");
        IntakeKoteru2 = hardwareMap.get(Servo.class, "IntakeKoteru2");
        Intake = hardwareMap.get(Servo.class, "Intake");
        IntakeKozgalu = hardwareMap.get(Servo.class, "IntakeKozgalu");

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");

        LiftBir = hardwareMap.get(DcMotor.class, "LiftBir");
        LiftEki = hardwareMap.get(DcMotor.class, "LiftEki");

        Oku1 = hardwareMap.get(DcMotor.class, "Oku1");
        Oku2 = hardwareMap.get(DcMotor.class, "Oku2");

        OutKoteru1 = hardwareMap.get(Servo.class, "OutKoteru1");
        OutKoteru2 = hardwareMap.get(Servo.class, "OutKoteru2");
        OutKoteru3 = hardwareMap.get(Servo.class, "OutKoteru3");
        OutKozgalu = hardwareMap.get(Servo.class, "OutKozgalu");
        Outtake = hardwareMap.get(Servo.class, "Outtake");


        //#######################################################################

        IntakeKoteru1.setDirection(Servo.Direction.FORWARD);
        IntakeKoteru2.setDirection(Servo.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        LiftBir.setDirection(DcMotor.Direction.FORWARD);
        LiftEki.setDirection(DcMotor.Direction.REVERSE);

        Oku1.setDirection(DcMotor.Direction.FORWARD);
        Oku2.setDirection(DcMotor.Direction.FORWARD);

        OutKoteru1.setDirection(Servo.Direction.FORWARD);
        OutKoteru2.setDirection(Servo.Direction.REVERSE);


        //####################################################################

        LiftBir.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftBir.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LiftEki.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftEki.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        Intake.setPosition(1);
        IntakeKoteru1.setPosition(0.3);
        IntakeKoteru2.setPosition(0.3);
        Outtake.setPosition(0);
        OutKoteru1.setPosition(0.1);
        OutKoteru2.setPosition(0.1);
        OutKozgalu.setPosition(0.35);
        OutKoteru3.setPosition(0.3);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Kozgalu();
                Intakee();
                Oku();
                LIft();
                Outtake();


                telemetry.update();
            }
        }


    }

    public void Kozgalu() {
        double yy = gamepad1.right_trigger;
        double qy = -gamepad1.left_trigger;
        double turn = gamepad1.left_stick_x;
        double x = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(qy) + Math.abs(yy) + Math.abs(x) + Math.abs(turn), 1);

        leftFront.setPower(((qy + yy + x + turn) / denominator));
        leftRear.setPower(((qy + yy - x + turn) / denominator));
        rightFront.setPower(((qy + yy - x - turn) / denominator));
        rightRear.setPower(((qy + yy + x - turn) / denominator) );
    }

    private void Intakee() {
        if (gamepad2.a && !qwe) {
            wqe = !wqe;
            qwe = true;
        } else if (!gamepad2.a) {
            qwe = false;
        }
        if (wqe) {
            IntakeKoteru1.setPosition(1);
            IntakeKoteru2.setPosition(1);
            if (!qwe) {
                Intake.setPosition(1);
            }
        } else {
            Intake.setPosition(0);
            if (!qwe) {
                IntakeKoteru1.setPosition(0.3);
                IntakeKoteru2.setPosition(0.3);
            }
        }
        double IntakeKozgaluu = -gamepad2.left_stick_x;
        IntakeKozgalu.setPosition((IntakeKozgaluu) + 0.5);
    }

        private void LIft() {
            double LiftAsty = gamepad2.left_trigger;
            double LiftTobe = gamepad2.right_trigger;
            double denominatorr = Math.max(Math.abs(LiftTobe) + Math.abs(LiftAsty), 1);
            LiftBir.setPower(((LiftTobe - LiftAsty) / denominatorr));
            LiftEki.setPower(((LiftTobe - LiftAsty) / denominatorr));
        }

        private void Oku() {
            double Okuu = -gamepad2.right_stick_y;
            double denominatorrr = Math.max(Math.abs(Okuu), 1);
            Oku1.setPower(Okuu / denominatorrr);
            Oku2.setPower(Okuu / denominatorrr);
        }


        private void Outtake () {
        int cPos = LiftBir.getCurrentPosition();
            if (gamepad1.a) {
                Outtake.setPosition(1);
                sleep(300);
                OutKoteru1.setPosition(0.1);
                OutKoteru2.setPosition(0.1);
                OutKozgalu.setPosition(0.35);
                OutKoteru3.setPosition(0.3);
            }
            if (gamepad1.b) {
                Outtake.setPosition(0);
                sleep(150);
                Intake.setPosition(1);
                sleep(150);
                if (cPos < TPos){
                    LiftBir.setPower(1);
                    LiftEki.setPower(1);
                }else {
                    LiftBir.setPower(0);
                    LiftEki.setPower(0);
                }
                OutKoteru1.setPosition(1);
                OutKoteru2.setPosition(1);
                OutKozgalu.setPosition(0.35);
                OutKoteru3.setPosition(0.65);
            }
        }
}
