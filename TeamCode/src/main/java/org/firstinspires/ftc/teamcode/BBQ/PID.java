package org.firstinspires.ftc.teamcode.BBQ;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double Kp, Ki, Kd;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double maxIntegralSum = 1.0;
    private double tolerance = 0;
    private double target;

    public PID(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        timer.reset();
    }

    public void setTarget(double target) {
        this.target = target;
        this.integralSum = 0;
        this.lastError = 0;
    }

    public double calculate(double current) {
        double error = target - current;

        if (Math.abs(error) <= tolerance) {
            return 0;
        }

        double P = Kp * error;

        integralSum += error * timer.seconds();
        if (integralSum > maxIntegralSum) integralSum = maxIntegralSum;
        if (integralSum < -maxIntegralSum) integralSum = -maxIntegralSum;
        double I = Ki * integralSum;

        double derivative = (error - lastError) / timer.seconds();
        double D = Kd * derivative;

        lastError = error;
        timer.reset();

        return P + I + D;
    }

    public void setTolerance(double newTolerance){
        tolerance = newTolerance;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public void setCoefficients(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }
}