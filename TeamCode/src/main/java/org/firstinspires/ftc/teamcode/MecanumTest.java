package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class MecanumTest extends LinearOpMode {
    private DcMotor LiftBir;

    private DcMotor ArtOn;
    private DcMotor ArtSolProb;
    private DcMotor AldOn;

    private DcMotor LiftEki;

    private DcMotor LiftEkii;
    private DcMotor AldSol;

    private Servo OutTake;
    private Servo OutKoteru;

    private Servo Intake;

    private Servo IntakeKoteru;

    private Servo IntakeKoteruu;

    private Servo IntakeAinaldyru;



    @Override
    public void runOpMode() {
        LiftBir = hardwareMap.get(DcMotor.class, "LiftBir");
        LiftEki = hardwareMap.get(DcMotor.class, "LiftEki");
        LiftEkii = hardwareMap.get(DcMotor.class, "LiftEkii");
        ArtOn = hardwareMap.get(DcMotor.class, "ArtOn");
        ArtSolProb = hardwareMap.get(DcMotor.class, "ArtSolProb");
        AldOn = hardwareMap.get(DcMotor.class, "AldOn");
        AldSol = hardwareMap.get(DcMotor.class, "AldSol");
        OutTake = hardwareMap.get(Servo.class, "OutTake");
        OutKoteru = hardwareMap.get(Servo.class, "OutKoteru");
        Intake = hardwareMap.get(Servo.class, "Intake");
        IntakeKoteru = hardwareMap.get(Servo.class, "IntakeKoteru");
        IntakeKoteruu = hardwareMap.get(Servo.class, "IntakeKoteruu");
        IntakeAinaldyru = hardwareMap.get(Servo.class, "IntakeAinaldyru");

        LiftBir.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftBir.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftEki.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftEki.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftEkii.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftEkii.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArtOn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArtOn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArtSolProb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArtSolProb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AldOn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AldOn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AldSol.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AldSol.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LiftBir.setDirection(DcMotor.Direction.FORWARD);
        LiftEki.setDirection(DcMotor.Direction.REVERSE);
        LiftEkii.setDirection(DcMotor.Direction.REVERSE);
        AldOn.setDirection(DcMotor.Direction.FORWARD);
        AldSol.setDirection(DcMotor.Direction.REVERSE);
        ArtOn.setDirection(DcMotor.Direction.FORWARD);
        ArtSolProb.setDirection(DcMotor.Direction.REVERSE);


        // Put initialization blocks here.
        waitForStart();
        IntakeAinaldyru.setPosition(0.5);
        OutTake.setPosition(0.5);
        OutKoteru.setPosition(0.5);


        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                Kozgaly1();
                LiftBir2();
                OutTake2();
                InTake2();
                LiftEki();
                telemetry.update();
            }
        }
    }

    private void Kozgaly1(){
        double qy = gamepad1.right_trigger;
        double yy = -gamepad1.left_trigger;
        double turn = gamepad1.left_stick_x;
        double x = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(qy) + Math.abs(yy) + Math.abs(x) + Math.abs(turn), 1);

        AldSol.setPower( (qy + yy + x + turn) / denominator);
        ArtSolProb.setPower( (qy + yy - x + turn) / denominator);
        AldOn.setPower( (qy + yy - x - turn) / denominator);
        ArtOn.setPower( (qy + yy + x - turn) / denominator);

    }
    private void LiftBir2() {
            double liftartka = gamepad2.right_trigger;
            double lift = gamepad2.left_trigger;
            double denominator = Math.max(Math.abs(lift) + Math.abs(liftartka) , 1);
            LiftBir.setPower((lift - liftartka) / denominator);
    }

    private void OutTake2(){
        if (gamepad1.b){
            OutTake.setPosition(0.1);
            //sleep(500);
            //Intake.setPosition(0.6);
        }
        if (gamepad1.a) {
            OutTake.setPosition(1);
        }
        if (gamepad2.x){
            OutKoteru.setPosition(0.1);
        }if (gamepad2.y) {
            OutKoteru.setPosition(0.985);
        }

    }
    private boolean isIntakeRunning = false;
    private long intakeStartTime = 0;

    private void InTake2() {
        if (gamepad2.dpad_left) {
            Intake.setPosition(0.5);
        }
        if (gamepad2.dpad_right && !isIntakeRunning) {
            Intake.setPosition(0.09);
            intakeStartTime = System.currentTimeMillis();
            isIntakeRunning = true;
        }

        if (isIntakeRunning) {
            if (System.currentTimeMillis() - intakeStartTime >= 600) {
                IntakeKoteru.setPosition(1);
                IntakeKoteruu.setPosition(0);
                LiftEki.setPower(-0.8);
                LiftEkii.setPower(-0.8);
                isIntakeRunning = false;
            }
        }

        if (gamepad2.dpad_up) {
            IntakeKoteru.setPosition(0.1);
            IntakeKoteruu.setPosition(0.9);
        }
        if (gamepad2.dpad_down) {
            IntakeKoteru.setPosition(1);
            IntakeKoteruu.setPosition(0);
        }

        double IntAinaldyruPower = -gamepad2.right_stick_x;
        double deadzone = 0.5;
        if (Math.abs(IntAinaldyruPower) < deadzone) {
            IntAinaldyruPower = 0;
        }
        IntAinaldyruPower = Math.max(-1, Math.min(1, IntAinaldyruPower));
        IntakeAinaldyru.setPosition(-IntAinaldyruPower + 0.5);
    }
    private void LiftEki(){
        double liftPower = -gamepad2.right_stick_y;
        double deadzone = 0.1;
        if (Math.abs(liftPower) < deadzone) {
            liftPower = 0;
        }
        liftPower = Math.max(-1, Math.min(1, liftPower));
        LiftEki.setPower(liftPower);
        LiftEkii.setPower(liftPower);
    }
  }

