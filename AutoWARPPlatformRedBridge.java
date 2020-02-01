package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


// WARP Dec 2019

@Autonomous
public class AutoWARPPlatformRedBridge extends LinearOpMode {
    private DcMotor front_left_wheel;
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_right_wheel;
    private DcMotor[] motors;

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
        Servo left_platform = hardwareMap.servo.get("left_platform");
        Servo right_platform = hardwareMap.servo.get("right_platform");
        left_platform.setDirection(Servo.Direction.FORWARD);
        right_platform.setDirection(Servo.Direction.REVERSE);
        left_platform.setPosition(0);
        right_platform.setPosition(0);

        // Lower the capstone servo
        Servo capstone = hardwareMap.servo.get("capstone");
        capstone.setDirection(Servo.Direction.REVERSE);
        capstone.setPosition(0);

        // wait for start button
        waitForStart();

        if (opModeIsActive()) {

            // Moving to platform.
            goRight(2445);
            goBack(450);


            left_platform.setPosition(0.4);
            right_platform.setPosition(0.45);
            sleep(1000);

            // Dragging platform
            goForward(100);
            rotateCW(425);
            goLeft(2000);
            rotateCW(1025);

            goBack(1915);
            rotateCW(400);
            left_platform.setPosition(0);
            right_platform.setPosition(0);
            sleep(1000);
            rotateCCW(100);

            // going under the bridge
            goLeft(3200);
            goBack(400);

        }
    }


    private void rotateCW(int position) {
        front_right_wheel.setTargetPosition(position);
        front_left_wheel.setTargetPosition(position);
        back_left_wheel.setTargetPosition(position);
        back_right_wheel.setTargetPosition(position);
        for (DcMotor motor : motors) {
            motor.setPower(.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((busyCounter() >= 2) && opModeIsActive()) {
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

    private void rotateCCW(int position) {
        front_right_wheel.setTargetPosition(-position);
        front_left_wheel.setTargetPosition(-position);
        back_left_wheel.setTargetPosition(-position);
        back_right_wheel.setTargetPosition(-position);
        for (DcMotor motor : motors) {
            motor.setPower(.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((busyCounter() >= 2) && opModeIsActive()) {
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


    private void goForward(int position) {
        front_right_wheel.setTargetPosition(-position);
        front_left_wheel.setTargetPosition(position);
        back_left_wheel.setTargetPosition(position);
        back_right_wheel.setTargetPosition(-position);
        for (DcMotor motor : motors) {
            motor.setPower(.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((busyCounter() >= 2) && opModeIsActive()) {
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

    private void goRight(int position) {
        front_right_wheel.setTargetPosition(position);
        front_left_wheel.setTargetPosition(position);
        back_left_wheel.setTargetPosition(-position);
        back_right_wheel.setTargetPosition(-position);
        for (DcMotor motor : motors) {
            motor.setPower(.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((busyCounter() >= 2) && opModeIsActive()) {
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

    private void goLeft(int position) {
        front_right_wheel.setTargetPosition(-position);
        front_left_wheel.setTargetPosition(-position);
        back_left_wheel.setTargetPosition(position);
        back_right_wheel.setTargetPosition(position);
        for (DcMotor motor : motors) {
            motor.setPower(.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((busyCounter() >= 2) && opModeIsActive()) {
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

    private void goBack(int position) {
        front_right_wheel.setTargetPosition(position);
        front_left_wheel.setTargetPosition(-position);
        back_left_wheel.setTargetPosition(-position);
        back_right_wheel.setTargetPosition(position);
        for (DcMotor motor : motors) {
            motor.setPower(.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((busyCounter() >= 2) && opModeIsActive()) {
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

    private int busyCounter() {
        int busyCount = 0;
        for (DcMotor motor : motors) {
            if (motor.isBusy()) {
                busyCount++;
            }
        }
        return busyCount;
    }
}

