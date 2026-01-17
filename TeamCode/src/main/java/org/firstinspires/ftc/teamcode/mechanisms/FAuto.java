package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FAuto {
    double integralSum1 = 0;

    public static double Kp1 = 2;
    public static double Ki1 = 0.1;
    public static double Kd1 = 0.3;

    public static double Kf1 = 0.1;
    ElapsedTime timer1 = new ElapsedTime();
    private double lastError1 = 0;

    //##########################PID2
    double integralSum2 = 0;

    public static double Kp2 = 2;
    public static double Ki2 = 0.1;
    public static double Kd2 = 0.3;

    public static double Kf2 = 0.1;
    ElapsedTime timer2 = new ElapsedTime();
    private double lastError2 = 0;




    //##################################
    private DcMotorEx Outtake1ex;
    private DcMotorEx Outtake2ex;

    private DcMotor Intake0ex;

    private DcMotor Trans3ex;
    //!!!!!!!!!!!!!!!!!!!!

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState{
        IDLE,
        SPIN_UP,
        RESET_GATE,
    }
    private enum IntakeState{
        InKos,
        Inson,
        Reset
    }

    private enum IntakeState1{
        InKos1,
        Inson1,
        Reset1
    }




    private FlywheelState flywheelState;

    private IntakeState intakeState;
    private IntakeState1 intakeState1;


    private double TransCloseTime = 0.6;
    private double TransCloseTime1 = 0.5;

    private double InOpenTime = 3.5;
    private double InOpenTime1 = 1;


    private int shotsRemaining = 0;
    private int intakeRemaining = 0;
    private int intakeRemaining1 = 0;
    private double ref = 2700;


    private double ShootTime = 3.5;

    public void init(HardwareMap hwMap){
        Trans3ex = hwMap.get(DcMotor.class, "Trans3ex");

        Intake0ex = hwMap.get(DcMotor.class, "Intake0ex");

        Outtake1ex = hwMap.get(DcMotorEx.class, "Outtake1ex");
        Outtake2ex = hwMap.get(DcMotorEx.class, "Outtake2ex");

        Outtake1ex.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake2ex.setDirection(DcMotorSimple.Direction.REVERSE);

        Outtake1ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake2ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake1ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outtake2ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelState = FlywheelState.IDLE;
        intakeState = IntakeState.InKos;
        intakeState1 = IntakeState1.InKos1;


    }
    public void update(){
        switch (flywheelState){
            case IDLE:
                if (shotsRemaining > 0){
                    Intake0ex.setPower(2000);
                    Trans3ex.setPower(-1000);
                    Outtake1ex.setVelocity(PIDcontrollForOut1(ref, Outtake1ex.getVelocity()));
                    Outtake2ex.setVelocity(PIDcontrollForOut2(ref, Outtake2ex.getVelocity()));
                    stateTimer.reset();

                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (stateTimer.seconds() > ShootTime){
                    Intake0ex.setPower(2000);
                    Trans3ex.setPower(1000);
                    shotsRemaining--;
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

                    else {
                        Intake0ex.setPower(0);
                        Trans3ex.setPower(0);
                        Outtake1ex.setPower(0);
                        Outtake2ex.setPower(0);

                        flywheelState = FlywheelState.IDLE;
                    }
                }
                break;
        }

    }

    public void intake(){
        switch (intakeState){
            case InKos:
                if (intakeRemaining > 0){
                    Intake0ex.setPower(4000);
                    Trans3ex.setPower(-1000);

                    stateTimer.reset();
                    intakeState = IntakeState.Inson;
                }
                break;

            case Inson:
                if (stateTimer.seconds() > InOpenTime){
                    intakeRemaining--;
                    Intake0ex.setPower(0);
                    Trans3ex.setPower(0);

                    stateTimer.reset();
                    intakeState = IntakeState.Reset;
                }
                break;

            case Reset:
                if (stateTimer.seconds() > TransCloseTime){
                    if (intakeRemaining > 0){
                        stateTimer.reset();
                        intakeState = IntakeState.Inson;
                    }

                    else {
                        intakeState = IntakeState.InKos;
                    }
                }
                break;

        }
    }
    public void intake1(){
        switch (intakeState1){
            case InKos1:
                if (intakeRemaining1 > 0){
                    Intake0ex.setPower(2000);
                    Trans3ex.setPower(-1000);

                    stateTimer.reset();
                    intakeState1 = IntakeState1.Inson1;
                }
                break;

            case Inson1:
                if (stateTimer.seconds() > InOpenTime1){
                    intakeRemaining1--;
                    Intake0ex.setPower(0);
                    Trans3ex.setPower(0);

                    stateTimer.reset();
                    intakeState1 = IntakeState1.Reset1;
                }
                break;

            case Reset1:
                if (stateTimer.seconds() > TransCloseTime1){
                    if (intakeRemaining1 > 0){
                        stateTimer.reset();
                        intakeState1 = IntakeState1.Inson1;
                    }

                    else {
                        intakeState1 = IntakeState1.InKos1;
                    }
                }
                break;

        }
    }


    public void FireShots(int number0fShots){
        if (flywheelState == FlywheelState.IDLE){
            shotsRemaining = number0fShots;
        }
    }

    public void InShots(int numberOfIn){
        if (intakeState == IntakeState.InKos){
            intakeRemaining = numberOfIn;
        }
    }

    public void InShots1(int numberOfIn1){
        if (intakeState1 == IntakeState1.InKos1){
            intakeRemaining1 = numberOfIn1;
        }
    }


    public boolean isBusy(){
        return flywheelState != FlywheelState.IDLE;
    }

    public double PIDcontrollForOut1(double reference1, double state1){
        double error1 = reference1 - state1;
        integralSum1 += error1 * timer1.seconds();
        double derivative1 = (error1 - lastError1) / timer1.seconds();
        lastError1 = error1;

        timer1.reset();

        double output1 = (error1 * Kp1) + (derivative1 * Kd1) + (integralSum1 * Ki1) + (reference1 * Kf1);
        return output1;


    }
    public double PIDcontrollForOut2(double reference2, double state2){
        double error2 = reference2 - state2;
        integralSum2 += error2 * timer2.seconds();
        double derivative2 = (error2 - lastError2) / timer2.seconds();
        lastError2 = error2;

        timer2.reset();

        double output2 = (error2 * Kp2) + (derivative2 * Kd2) + (integralSum2 * Ki2) + (reference2 * Kf2);
        return output2;


    }

}
