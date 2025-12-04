package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebCam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class testInOut extends OpMode {

    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    private DcMotor Intake0;

    private DcMotor Outtake1;
    private DcMotor Outtake2;

    private Servo xy0;
    private Servo xy1;

    private Servo z2;

    private Servo o4;
    private Servo o5;

    private DcMotor lfex0;
    private DcMotor lrex1;
    private DcMotor rfex2;
    private DcMotor rrex3;

    final double kp = 0.0040;
    boolean tftrans = false;
    boolean tftrans1 = true;

    boolean tfin = false;
    boolean tfin2 = true;

    boolean tfout1 = false;
    boolean tfout2 = true;

    int i1 = 0;
    int i3 = 0;

    int i2 = 0;





    @Override
    public void init() {
        aprilTagWebCam.init(hardwareMap, telemetry);

        Intake0 = hardwareMap.get(DcMotor.class, "Intake0");

        Outtake1 = hardwareMap.get(DcMotor.class, "Outtake1");
        Outtake2 = hardwareMap.get(DcMotor.class, "Outtake2");

        xy0 = hardwareMap.get(Servo.class, "xy0");
        xy1 = hardwareMap.get(Servo.class, "xy1");

        z2 = hardwareMap.get(Servo.class, "z2");


        o4 = hardwareMap.get(Servo.class, "o4");
        o5 = hardwareMap.get(Servo.class, "o5");

        lfex0 = hardwareMap.get(DcMotor.class, "lfex0");
        lrex1 = hardwareMap.get(DcMotor.class, "lrex1");
        rfex2 = hardwareMap.get(DcMotor.class, "rfex2");
        rrex3 = hardwareMap.get(DcMotor.class, "rrex3");


        Outtake1.setDirection(DcMotorSimple.Direction.FORWARD);
        Outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake0.setDirection(DcMotorSimple.Direction.REVERSE);

        lfex0.setDirection(DcMotorSimple.Direction.FORWARD);
        lrex1.setDirection(DcMotorSimple.Direction.FORWARD);
        rfex2.setDirection(DcMotorSimple.Direction.FORWARD);
        rrex3.setDirection(DcMotorSimple.Direction.REVERSE);


        o4.setPosition(1);
        o5.setPosition(0);

    }



    @Override
    public void loop() {
        Outtake();
        OutCam();
        Intake();
        meka();
        Trnans();
        double xrrr = gamepad2.right_stick_x;
        xy0.setPosition(xrrr + 0.5);
        xy1.setPosition(xrrr + 0.5);


    }



    public void meka() {
        double yy = gamepad1.right_trigger;
        double qy = -gamepad1.left_trigger;
        double turn = gamepad1.left_stick_x;
        double x = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(qy) + Math.abs(yy) + Math.abs(x) + Math.abs(turn), 1);

        lfex0.setPower((qy + yy + x + turn) / denominator);
        lrex1.setPower((qy + yy - x + turn) / denominator);
        rfex2.setPower((qy + yy - x - turn) / denominator);
        rrex3.setPower((qy + yy + x - turn) / denominator);
    }



    public void Outtake() {

        if (gamepad2.b & !tfout1) {
            tfout2 = false;
            if (Outtake1.getPower() == 1) {
                tfout1 = true;
                Outtake1.setPower(0);
                Outtake2.setPower(0);
                tfout2 = true;
            } else if (Outtake1.getPower() == 0) {
                tfout1 = true;
                Outtake1.setPower(6000);
                Outtake2.setPower(6000);
                tfout2 = true;
            }
        }else if (!gamepad2.b && tfout2) {
            tfout1 = false;
        }
    }



    public void Intake() {

        if (gamepad2.a & !tfin) {
            tfin2 = false;
            if (Intake0.getPower() == 0) {
                Intake0.setPower(6000);
                tfin = true;
                tfin2 = true;
            } else if (Intake0.getPower() == 1) {
                Intake0.setPower(0);
                tfin = true;
                tfin2 = true;
            }
        } else if (!gamepad2.a && tfin2) {
            tfin = false;
        }
    }



    public void Trnans() {
        if (gamepad1.a & !tftrans) {
            tftrans1 = false;
            if (o4.getPosition() == 1) {
                o4.setPosition(0.5);
                tftrans = true;
                for(i1=1; i1<30000000; i1++){}
                if (o4.getPosition() == 0.5 && i1 == 30000000) {
                    o5.setPosition(0.7);
                    tftrans1 = true;
                }
            } else if (o4.getPosition() == 0.5) {
                o4.setPosition(1);
                tftrans = true;
                for (i2 = 1; i2<1000000; i2++){}
                if (o4.getPosition() == 1 && i2 == 1000000) {
                    o5.setPosition(0);
                    tftrans1 = true;
                }
            }
        } else if (!gamepad1.a && tftrans1) {
            tftrans = false;
        }
    }



    public void OutCam(){
        aprilTagWebCam.update();
        AprilTagDetection tag = aprilTagWebCam.getTagSpecificId(24);
        aprilTagWebCam.displayDetectionTelemetry(tag);

        if(tag != null){
            double camy = tag.ftcPose.y;
            double z2pos = (0.6 - (camy * kp));
            z2.setPosition(z2pos);

        }


        telemetry.update();
    }
}
