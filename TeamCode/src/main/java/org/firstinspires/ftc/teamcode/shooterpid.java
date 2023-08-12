package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class shooterpid extends LinearOpMode {
    DcMotorEx shooter;
    double integralSum = 0;
    double Kp = 6.8;
    double Ki = 7.7;
    double Kd = 10;
    double kf = 10;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            double shooterPower = PIDControl(1000, shooter.getCurrentPosition());
            shooter.setVelocity(6000);
        }
    }
    public double PIDControl(double reference, double state){
        double error = reference - state;

        integralSum += error + timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        lastError = error;


        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki)+ (reference * kf);
        return output;
    }

}