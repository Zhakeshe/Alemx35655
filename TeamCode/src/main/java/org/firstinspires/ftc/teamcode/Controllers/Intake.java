package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    private DcMotor Intake, Intake2;
    private Servo Stopper;

    public static double SERVO_OPEN_POS = 0.4;
    public static double SERVO_CLOSED_POS = 0.05;

    public enum IntakeState {
        OPEN,
        CLOSED
    }

    public void initialize(HardwareMap hardwareMap) {
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Intake2 = hardwareMap.get(DcMotor.class, "transfer");

        Stopper = hardwareMap.get(Servo.class, "stopper");

        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        setServo(IntakeState.CLOSED);
    }



    public void setServo(IntakeState state) {
        double position = (state == IntakeState.OPEN) ? SERVO_OPEN_POS : SERVO_CLOSED_POS;

        position = Math.max(0.0, Math.min(1.0, position));

        Stopper.setPosition(position);
    }
    public void setIntakePower(double power) {
        Intake.setPower(power);
        Intake2.setPower(power);
    }

    public void setCustomPosition(double pos) {
        Stopper.setPosition(pos);
    }
}