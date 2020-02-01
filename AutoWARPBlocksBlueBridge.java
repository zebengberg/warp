package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


// WARP Dec 2019

@Autonomous
public class AutoWARPBlocksBlueBridge extends LinearOpMode {
    private DcMotor front_left_wheel;
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_right_wheel;
    private DcMotor[] motors;

    // Creating a macro for speed of DC motors.
    private double speed = 0.75;

    private ColorSensor right_color;


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

        // Servos for little arms
        Servo right_arm = hardwareMap.servo.get("right_arm");
        right_arm.setDirection(Servo.Direction.REVERSE);

        // Sensors
        right_color = hardwareMap.get(ColorSensor.class, "right_color");

        // Lower the capstone servo in case it was left up
        Servo capstone = hardwareMap.servo.get("capstone");
        capstone.setDirection(Servo.Direction.REVERSE);
        capstone.setPosition(0);


        waitForStart();

        if (opModeIsActive()) {

            // Going to the blocks initially.
            rotateCW(30);
            goLeft(2360);

            // Finding the first black block. Variable will change to 1, 2, or 3
            int block_count = 1;
            while (block_count < 3) {  // condition to guarantee we don't get stuck here if sensor fails
                if (isYellow()) {
                    goForward(710);
                    rotateCW(30);
                    block_count++;
                } else {
                    break;
                }
            }

            // Grabbing the black block.
            right_arm.setPosition(0.88);
            sleep(1500);

            // Moving back to wall
            goRight(1200);

            switch (block_count) {
                case 1:
                    // Going under the skystone bridge. Releasing block. Moving back toward fence a hair.
                    goBack(3500);
                    right_arm.setPosition(.1);
                    sleep(500);
                    goRight(200);
                    rotateCCW(30);

                    // Going back to the blocks.
                    goForward(5200);
                    goLeft(1180);

                    // Grabbing the second black block and moving back toward fence.
                    right_arm.setPosition(0.88);
                    sleep(1000);
                    goRight(900);
                    rotateCCW(30);

                    // Going under the skystone bridge and releasing block.
                    goBack(5580);
                    right_arm.setPosition(.1);
                    sleep(500);
                    goRight(200);

                    // Going to sky bridge tape.
                    goForward(1300);
                    goLeft(1200);
                    break;

                case 2:
                    // Going under the skystone bridge. Releasing block. Moving back toward fence a hair.
                    goBack(4210);
                    right_arm.setPosition(.1);
                    sleep(500);
                    goRight(200);
                    rotateCCW(50);

                    // Going back to the blocks.
                    goForward(6100);
                    goLeft(1200);
                    rotateCCW(40);

                    // Grabbing the black block and moving back toward fence.
                    right_arm.setPosition(0.88);
                    sleep(1000);
                    goRight(800);
                    rotateCCW(30);

                    // Going under the skystone bridge and releasing block.
                    goBack(6100);
                    right_arm.setPosition(.1);
                    sleep(500);
                    goRight(200);

                    // Going to sky bridge tape.
                    goForward(1100);
                    goLeft(1000);
                    break;

                case 3:
                    // Going under the skystone bridge. Releasing block. Moving back toward fence a hair.
                    goBack(4500);
                    right_arm.setPosition(.1);
                    sleep(500);
                    goRight(200);
                    rotateCCW(65);

                    // Going back to the blocks.
                    goForward(6400);
                    goLeft(1350);

                    // Grabbing the second black block and moving back toward fence.
                    right_arm.setPosition(0.88);
                    sleep(1000);
                    goRight(1200);
                    rotateCCW(50);

                    // Going under the skystone bridge and releasing block.
                    goBack(7000);
                    right_arm.setPosition(.1);
                    sleep(500);
                    goRight(200);

                    // Going to sky bridge tape.
                    goForward(1400);
                    goLeft(800);
                    break;
            }
        }
    }

    private boolean isYellow() {
        float[] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(right_color.red() * 255, right_color.green() * 255, right_color.blue() * 255, hsv);
        float hue = hsv[0];  // sat = hsv[1] and val = hsv[2]
        return (hue < 90);
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
            motor.setPower(speed);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((busyCounter() >= 2) && opModeIsActive()) {
            telemetry.addData("target position", position);
            telemetry.addData("front left", front_left_wheel.getCurrentPosition());
            telemetry.addData("front right", front_right_wheel.getCurrentPosition());
            telemetry.addData("back left", back_left_wheel.getCurrentPosition());
            telemetry.addData("back right", back_right_wheel.getCurrentPosition());
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
            motor.setPower(speed);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((busyCounter() >= 2) && opModeIsActive()) {
            telemetry.addData("target position", position);
            telemetry.addData("front left", front_left_wheel.getCurrentPosition());
            telemetry.addData("front right", front_right_wheel.getCurrentPosition());
            telemetry.addData("back left", back_left_wheel.getCurrentPosition());
            telemetry.addData("back right", back_right_wheel.getCurrentPosition());
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
            motor.setPower(speed);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((busyCounter() >= 2) && opModeIsActive()) {
            telemetry.addData("target position", position);
            telemetry.addData("front left", front_left_wheel.getCurrentPosition());
            telemetry.addData("front right", front_right_wheel.getCurrentPosition());
            telemetry.addData("back left", back_left_wheel.getCurrentPosition());
            telemetry.addData("back right", back_right_wheel.getCurrentPosition());
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
            motor.setPower(speed);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((busyCounter() >= 2) && opModeIsActive()) {
            telemetry.addData("target position", position);
            telemetry.addData("front left", front_left_wheel.getCurrentPosition());
            telemetry.addData("front right", front_right_wheel.getCurrentPosition());
            telemetry.addData("back left", back_left_wheel.getCurrentPosition());
            telemetry.addData("back right", back_right_wheel.getCurrentPosition());
            telemetry.update();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}