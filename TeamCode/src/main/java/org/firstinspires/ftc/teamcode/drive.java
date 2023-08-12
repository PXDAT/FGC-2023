package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptGamepadRumble;


@TeleOp(name = "Main")
public class drive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public boolean state = false, state1= false;
    private Servo hook1 = null;
    private Servo hook2 = null;
    private Servo door = null;
    private CRServo storage_lift = null;
    private DcMotor climber = null, intake = null, bot_lift = null;
    
    DcMotorEx shooter;
    double integralSum = 0;
    double Kp = 6.8, Ki = 7.7, Kd = 10, Kf = 10;
    int count = 0;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    @Override
    public void runOpMode() {
//        core hex
        climber = hardwareMap.get(DcMotor.class, "climber");
        bot_lift = hardwareMap.get(DcMotor.class,"bot_lift");
//        HD motor
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
//        servo setup
        hook1 = hardwareMap.get(Servo.class,"hook1");
        hook2 = hardwareMap.get(Servo.class, "hook2");
        door = hardwareMap.get(Servo.class, "door");
        storage_lift = hardwareMap.get(CRServo.class, "storage_lift");
//        Motor driver set direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
//        PID setup
        shooter.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // tank drive with joystick
            leftFrontDrive.setPower(gamepad1.left_stick_y);
            rightFrontDrive.setPower(-gamepad1.right_stick_y);
            leftBackDrive.setPower(gamepad1.left_stick_y);
            rightBackDrive.setPower(-gamepad1.right_stick_y);
            if(gamepad1.left_trigger == 0 & gamepad1.right_trigger == 0)
            {
                hook1.setPosition(0.5);
                hook2.setPosition(0.5);
            }
            else if(gamepad1.left_trigger > 0)
            {
                hook1.setPosition(hook1.getPosition()+0.05);
                hook2.setPosition(hook2.getPosition()+0.05);
                leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            }
            else if(gamepad1.right_trigger > 0)
            {
                hook1.setPosition(hook1.getPosition()-0.05);
                hook2.setPosition(hook2.getPosition()-0.05);
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            }

            if(gamepad1.left_bumper)
            {
                count++;
            }
            switch(count % 2)
            {
                case(1):
                    double shooterPower = PIDControl(1000, shooter.getCurrentPosition());
                    shooter.setVelocity(2000);
                    break;
                case(0):
                    shooter.setVelocity(0);
                    break;
            }
            if(gamepad1.right_bumper)
            {
                state1 =! state1;
            }
            if(state1)
            {
                intake.setPower(1);
            }
            else
            {
                intake.setPower(0);
            }
            if(gamepad1.dpad_up)
            {
                climber.setPower(1);
            }
            else if(gamepad1.dpad_down)
            {
                climber.setPower(-1);
            }
            else
            {
                climber.setPower(0);
            }
            if(gamepad1.square)
            {
                bot_lift.setPower(1);
            }
            else if(gamepad1.circle)
            {
                bot_lift.setPower(-1);
            }
            else
            {
                bot_lift.setPower(0);
            }
            if(gamepad1.dpad_left)
            {
                door.setPosition(door.getPosition()+0.1);
            }
            if(gamepad1.dpad_right)
            {
                door.setPosition(door.getPosition()-0.1);
            }
            else
            {
                door.setPosition(0.5);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
//    PID CONTROL
    public double PIDControl(double reference, double state){
        double error = reference - state;

        integralSum += error + timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        lastError = error;


        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki)+ (reference * Kf);
        return output;
    }
}
