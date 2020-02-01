package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


// WARP Dec 2019

@Autonomous
public class AutoWARPBlocksRedBridge extends LinearOpMode {
    private DcMotor front_left_wheel;
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_right_wheel;
    private DcMotor[] motors;

    // Creating a macro for speed of DC motors.
    private double speed = 0.7;

    private ColorSensor left_color;





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
        Servo left_arm = hardwareMap.servo.get("left_arm");
        left_arm.setDirection(Servo.Direction.FORWARD);
        left_arm.setPosition(.1);

        // Sensors
        left_color = hardwareMap.get(ColorSensor.class, "left_color");

        // Lower the capstone servo
        Servo capstone = hardwareMap.servo.get("capstone");
        capstone.setDirection(Servo.Direction.REVERSE);
        capstone.setPosition(0);


        // wait for start button
        waitForStart();

        if (opModeIsActive()) {

            // Going to the blocks initially. Finding the first black block.
            int block_count = 0;  // will change to 1, 2, or 3
            goLeft(2280);
            while (block_count < 2) {  // condition to guarantee we don't get stuck here if sensor fails
                if (isYellow()) {
                    rotateCW(30);
                    goBack(710);
                    block_count++;
                } else {
                    block_count++;
                    break;
                }
            }

            // Grabbing the black block.
            left_arm.setPosition(0.68);
            sleep(1500);

            // Moving back to wall
            goRight(800);
            rotateCW(30);

            switch (block_count) {
                case 1:
                    // Going under the skystone bridge. Releasing block.
                    goForward(3000);
                    left_arm.setPosition(0);
                    sleep(500);
                    rotateCW(30);

                    // Going back to the blocks.
                    goBack(4800);
                    goLeft(1150);

                    // Grabbing the black block and
                    rotateCCW(100);
                    left_arm.setPosition(0.68);
                    sleep(1000);

                    // Moving back toward fence
                    goRight(800);
                    rotateCW(100);

                    // Going under the skystone bridge and releasing block.
                    goForward(5130);
                    left_arm.setPosition(0);
                    sleep(500);
                    goRight(200);

                    // Going to sky bridge tape.
                    goBack(1100);
                    goLeft(1000);
                    break;

                case 2:
                    // Going under the skystone bridge. Releasing block.
                    goForward(3710);
                    left_arm.setPosition(0);
                    sleep(500);
                    rotateCW(30);

                    // Going back to the blocks.
                    goBack(5840);
                    goLeft(1150);

                    // Grabbing the black block and
                    rotateCCW(100);
                    left_arm.setPosition(0.68);
                    sleep(1000);

                    // Moving back toward fence
                    goRight(800);
                    rotateCW(100);

                    // Going under the skystone bridge and releasing block.
                    goForward(5840);
                    left_arm.setPosition(0);
                    sleep(500);
                    goRight(200);

                    // Going to sky bridge tape.
                    goBack(1100);
                    goLeft(1000);
                    break;

                case 3:
                    // Going under the skystone bridge. Releasing block.
                    goForward(4420);
                    left_arm.setPosition(0);
                    sleep(500);
                    rotateCW(30);

                    // Going back to the blocks.
                    goBack(6550);
                    goLeft(1150);

                    // Grabbing the black block and
                    rotateCCW(100);
                    left_arm.setPosition(0.68);
                    sleep(1000);

                    // Moving back toward fence
                    goRight(800);
                    rotateCW(100);

                    // Going under the skystone bridge and releasing block.
                    goForward(6550);
                    left_arm.setPosition(0);
                    sleep(500);
                    goRight(200);

                    // Going to sky bridge tape.
                    goBack(1100);
                    goLeft(1000);
                    break;
            }
        }
    }

    private boolean isYellow() {
        float[] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(left_color.red() * 255, left_color.green() * 255, left_color.blue() * 255, hsv);
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