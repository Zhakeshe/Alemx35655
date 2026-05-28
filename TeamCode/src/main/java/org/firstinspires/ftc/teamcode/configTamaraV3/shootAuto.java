package org.firstinspires.ftc.teamcode.configTamaraV3;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configTamaraV3.shootAuto;

public class shootAuto {

    private DcMotorEx shooter1, shooter2;
    private DcMotor intake, transfer;

    double integralSum1 = 0;

    public static double Kp1 = 0.006;
    public static double Ki1 = 0;
    public static double Kd1 = 0.00005;

    public static double Kf1 = 0.9;
    ElapsedTime timer1 = new ElapsedTime();
    private double lastError1 = 0;


    public static double ref = 2100;

    double targetRef = 0;

    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime inTimer = new ElapsedTime();

    private int shotsRemaining = 0;
    private double ShootTime = 0.5;
    private double TransCloseTime = 2;

    private int intakeRemaining = 0;
    private double IntakeTime = 4;
    private double IntakeCloseTime = 0.4;


    private enum FlywheelState{
        IDLE,
        SPIN_UP,
        RESET_GATE
    }
    private enum IntakeState{
        InKos,
        Inson,
        Reset
    }
    private FlywheelState flywheelState;
    private IntakeState intakeState;

    public void init(HardwareMap hwMap){
        intake = hwMap.get(DcMotor.class, "intake");
        transfer = hwMap.get(DcMotor.class, "transfer");
        shooter1 = hwMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hwMap.get(DcMotorEx.class, "shooter2");


        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelState = FlywheelState.IDLE;
        intakeState = IntakeState.InKos;
    }

    public void update(){
        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    intake.setPower(1);
                    transfer.setPower(-1);
                    shooter1.setVelocity(PIDcontrollHigh1(ref, shooter1.getVelocity()));
                    shooter2.setVelocity(PIDcontrollHigh1(ref, shooter1.getVelocity()));
                    stateTimer.reset();

                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (stateTimer.seconds() > ShootTime){
                    intake.setPower(1);
                    transfer.setPower(1);
                    shooter1.setVelocity(PIDcontrollHigh1(ref, shooter1.getVelocity()));
                    shooter2.setVelocity(PIDcontrollHigh1(ref, shooter1.getVelocity()));
                    shotsRemaining --;
                    stateTimer.reset();

                    flywheelState = FlywheelState.RESET_GATE;
                }
                break;
            case RESET_GATE:
                if (stateTimer.seconds() > TransCloseTime){
                    if (shotsRemaining > 0){
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    }

                    else{
                        intake.setPower(0);
                        transfer.setPower(0);
                        shooter1.setPower(0);
                        shooter2.setPower(0);
                        flywheelState = FlywheelState.IDLE;
                    }
                }
                break;
        }
    }

    public void intake(){
        switch (intakeState) {
            case InKos:
                if (intakeRemaining > 0) {
                    intake.setPower(1);
                    transfer.setPower(-1);
                    inTimer.reset();
                    intakeState = IntakeState.Inson;
                }
                break;


            case Inson:
                if (inTimer.seconds() > IntakeTime){
                    intakeRemaining--;
                    intake.setPower(0);
                    transfer.setPower(0);

                    inTimer.reset();
                    intakeState = IntakeState.Reset;
                }
                break;
            case Reset:
                if (inTimer.seconds() > IntakeCloseTime){
                    if (intakeRemaining > 0){
                        inTimer.reset();
                        intakeState = IntakeState.Inson;
                    }
                    else{
                        intakeState = IntakeState.InKos;
                    }
                }
                break;
        }
    }

    public void FireShots(int number0fShots){
        if (flywheelState == shootAuto.FlywheelState.IDLE){
            shotsRemaining = number0fShots;
        }
    }

    public void InShotes(int numberOfIn){
        if (intakeState == IntakeState.InKos){
            intakeRemaining = numberOfIn;
        }
    }


    public boolean isBusy(){
        return flywheelState != shootAuto.FlywheelState.IDLE;
    }




    public double PIDcontrollHigh1(double reference1, double state1){
        double error1 = reference1 - state1;
        integralSum1 += error1 * timer1.seconds();
        double derivative1 = (error1 - lastError1) / timer1.seconds();
        lastError1 = error1;

        timer1.reset();

        return (error1 * Kp1) + (derivative1 * Kd1) + (integralSum1 * Ki1) + (reference1 * Kf1);


    }
}
