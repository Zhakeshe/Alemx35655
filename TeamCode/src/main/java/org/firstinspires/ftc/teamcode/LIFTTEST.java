package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "LIFTTEST", group = "examples")
public class LIFTTEST extends LinearOpMode {
    private DcMotor LiftBir;
    private DcMotor LiftEki;

    private Servo IntakeKozgalu;

    private Servo IntakeKozgalu2;

    private Servo Intake;

    private Servo IntakeKoteru1;
    private Servo IntakeKoteru2;

    private DcMotor AldOn;
    private DcMotor AldSol;
    private DcMotor ArtOn;

    private DcMotor ArtSol;

    private Servo Outtake;

    private Servo OutKoteru1;
    private Servo OutKoteru2;




    @Override
    public void runOpMode() {
        LiftBir = hardwareMap.get(DcMotor.class, "LiftBir");
        LiftEki = hardwareMap.get(DcMotor.class, "LiftEki");

        IntakeKozgalu = hardwareMap.get(Servo.class, "IntakeKozgalu");
        IntakeKozgalu2 = hardwareMap.get(Servo.class, "IntakeKozgalu2");
        Intake = hardwareMap.get(Servo.class, "Intake");

        IntakeKoteru1 = hardwareMap.get(Servo.class, "IntakeKoteru1");
        IntakeKoteru2 = hardwareMap.get(Servo.class, "IntakeKoteru2");

        AldOn = hardwareMap.get(DcMotor.class, "AldOn");
        AldSol = hardwareMap.get(DcMotor.class, "AldSol");
        ArtOn = hardwareMap.get(DcMotor.class, "ArtOn");
        ArtSol = hardwareMap.get(DcMotor.class, "ArtSol");


        Outtake = hardwareMap.get(Servo.class, "Outtake");

        OutKoteru1 = hardwareMap.get(Servo.class, "OutKoteru1");
        OutKoteru2 = hardwareMap.get(Servo.class, "OutKoteru2");


        LiftBir.setDirection(DcMotor.Direction.FORWARD);
        LiftEki.setDirection(DcMotor.Direction.REVERSE);

        AldOn.setDirection(DcMotorSimple.Direction.FORWARD);
        AldSol.setDirection(DcMotorSimple.Direction.REVERSE);
        ArtOn.setDirection(DcMotorSimple.Direction.FORWARD);
        ArtSol.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        Intake.setPosition(0.7);
        IntakeKoteru1.setPosition(0.8);
        IntakeKoteru2.setPosition(0.2);
        IntakeKozgalu2.setPosition(0);
        Outtake.setPosition(1);
        OutKoteru1.setPosition(0.83);
        OutKoteru2.setPosition(0.17);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Kozgalu();
                LIft();
                IntakeKozgaluu();
                Intakee();
                OutTakee();

            }
        }
    }


    public void Kozgalu(){
        double qy = gamepad1.right_trigger;
        double yy = -gamepad1.left_trigger;
        double turn = -gamepad1.left_stick_x;
        double x = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(qy) + Math.abs(yy) + Math.abs(x) + Math.abs(turn), 1);

        AldSol.setPower( (qy + yy + x + turn) / denominator);
        ArtSol.setPower( (qy + yy - x + turn) / denominator);
        AldOn.setPower( (qy + yy - x - turn) / denominator);
        ArtOn.setPower( (qy + yy + x - turn) / denominator);
    }


    private void LIft(){
        double LiftAsty = gamepad2.left_trigger;
        double LiftTobe = gamepad2.right_trigger;
        double denominatorr = Math.max(Math.abs(LiftTobe) + Math.abs(LiftAsty) , 1);
        LiftBir.setPower((LiftTobe - LiftAsty ) / denominatorr);
        LiftEki.setPower((LiftTobe - LiftAsty ) / denominatorr);
    }


    private void IntakeKozgaluu(){
        double IntakeKozgaluu = -gamepad2.left_stick_x;
        IntakeKozgalu.setPosition((IntakeKozgaluu)+0.5);
    }


    private void Intakee(){
        if(gamepad2.a){
            Intake.setPosition(0.2);
            sleep(500);
            IntakeKozgalu2.setPosition(1);
            sleep(300);
            IntakeKoteru1.setPosition(0.3);
            IntakeKoteru2.setPosition(0.7);
        }
        if(gamepad2.b) {
            Intake.setPosition(0.7);
            sleep(500);
            IntakeKoteru1.setPosition(0.8);
            IntakeKoteru2.setPosition(0.2);
            sleep(300);
            IntakeKozgalu2.setPosition(0);
        }
    }

    private void OutTakee(){
        if(gamepad1.b){
            Outtake.setPosition(1);
            sleep(400);
            Intake.setPosition(0.2);
            OutKoteru1.setPosition(0.30);
            OutKoteru2.setPosition(0.70);
        }
        if (gamepad1.a){
            Outtake.setPosition(0);
            sleep(300);
            OutKoteru1.setPosition(0.83);
            OutKoteru2.setPosition(0.17);
        }
    }

}
