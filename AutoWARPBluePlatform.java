package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


// WARP Dec 2019

@Autonomous
public class AutoWARPBluePlatform extends LinearOpMode {
    DcMotor front_left_wheel;
    DcMotor back_left_wheel;
    DcMotor back_right_wheel;
    DcMotor front_right_wheel;
    DcMotor[] motors;

    Servo left_platform;
    Servo right_platform;



    @Override
    public void runOpMode() {
        // DC motors for holonomic drive.
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");

        // Creating an array of motors so we can iterate over it.
        motors = new DcMotor[] {back_left_wheel, back_right_wheel, front_right_wheel, front_left_wheel};

        // Initializing the motors.
        for (DcMotor motor : motors) {
            // REV HD Hex encoder counts 2240 per rotation.
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Servos for moving the platform.
        left_platform = hardwareMap.servo.get("left_platform");
        right_platform = hardwareMap.servo.get("right_platform");
        left_platform.setDirection(Servo.Direction.FORWARD);
        right_platform.setDirection(Servo.Direction.REVERSE);
        left_platform.setPosition(0);
        right_platform.setPosition(0);

        // wait for start button
        waitForStart();

        if (opModeIsActive()) {
            goRight(2650);
            left_platform.setPosition(0.4);
            right_platform.setPosition(0.5);
            sleep(1000);

            goLeft(3000);
            left_platform.setPosition(0);
            right_platform.setPosition(0);
            sleep(1000);
            goLeft(100);


            goBack(3000);
            goLeft(200);
            goBack(800);
            goLeft(200);
        }
    }



    public void goForward(int position) {
        front_right_wheel.setTargetPosition(-position);
        front_left_wheel.setTargetPosition(position);
        back_left_wheel.setTargetPosition(position);
        back_right_wheel.setTargetPosition(-position);
        for (DcMotor motor : motors) {
            motor.setPower(.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((front_left_wheel.isBusy() || front_right_wheel.isBusy() || back_right_wheel.isBusy()
                || back_left_wheel.isBusy()) && opModeIsActive()) {
            telemetry.addData("target position", position);
            for (DcMotor motor : motors) {
                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
            }
            telemetry.update();
        }

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void goRight(int position) {
        front_right_wheel.setTargetPosition(position);
        front_left_wheel.setTargetPosition(position);
        back_left_wheel.setTargetPosition(-position);
        back_right_wheel.setTargetPosition(-position);
        for (DcMotor motor : motors) {
            motor.setPower(.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((front_left_wheel.isBusy() || front_right_wheel.isBusy() || back_right_wheel.isBusy()
                || back_left_wheel.isBusy()) && opModeIsActive()) {
            telemetry.addData("target position", position);
            for (DcMotor motor : motors) {
                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
            }
            telemetry.update();
        }

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void goLeft(int position) {
        front_right_wheel.setTargetPosition(-position);
        front_left_wheel.setTargetPosition(-position);
        back_left_wheel.setTargetPosition(position);
        back_right_wheel.setTargetPosition(position);
        for (DcMotor motor : motors) {
            motor.setPower(.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((front_left_wheel.isBusy() || front_right_wheel.isBusy() || back_right_wheel.isBusy()
                || back_left_wheel.isBusy()) && opModeIsActive()) {
            telemetry.addData("target position", position);
            for (DcMotor motor : motors) {
                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
            }
            telemetry.update();
        }

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void goBack(int position) {
        front_right_wheel.setTargetPosition(position);
        front_left_wheel.setTargetPosition(-position);
        back_left_wheel.setTargetPosition(-position);
        back_right_wheel.setTargetPosition(position);
        for (DcMotor motor : motors) {
            motor.setPower(.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((front_left_wheel.isBusy() || front_right_wheel.isBusy() || back_right_wheel.isBusy()
                || back_left_wheel.isBusy()) && opModeIsActive()) {
            telemetry.addData("target position", position);
            for (DcMotor motor : motors) {
                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
            }
            telemetry.update();
        }

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
