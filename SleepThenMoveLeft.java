package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// WARP Dec 2019

@Autonomous
public class SleepThenMoveLeft extends LinearOpMode {
    DcMotor front_left_wheel;
    DcMotor back_left_wheel;
    DcMotor back_right_wheel;
    DcMotor front_right_wheel;
    DcMotor[] motors;


    @Override
    public void runOpMode() {
        // DC motors for holonomic drive.
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");
        motors = new DcMotor[] {back_left_wheel, back_right_wheel, front_right_wheel, front_left_wheel};

        for (DcMotor motor : motors) {
            // REV HD Hex encoder counts 2240 per rotation.
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        // wait for start button
        waitForStart();

        if (opModeIsActive()) {
            sleep(27000);
            goForward(200);
            goLeft(3000);
        }
    }



    public void goForward(int position) {
        front_right_wheel.setTargetPosition(-position);
        front_left_wheel.setTargetPosition(position);
        back_left_wheel.setTargetPosition(position);
        back_right_wheel.setTargetPosition(-position);
        for (DcMotor motor : motors) {
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((front_left_wheel.isBusy() || front_right_wheel.isBusy() || back_right_wheel.isBusy()
                || back_left_wheel.isBusy()) && opModeIsActive()) {
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

    public void goRight(int position) {
        front_right_wheel.setTargetPosition(position);
        front_left_wheel.setTargetPosition(position);
        back_left_wheel.setTargetPosition(-position);
        back_right_wheel.setTargetPosition(-position);
        for (DcMotor motor : motors) {
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((front_left_wheel.isBusy() || front_right_wheel.isBusy() || back_right_wheel.isBusy()
                || back_left_wheel.isBusy()) && opModeIsActive()) {
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

    public void goLeft(int position) {
        front_right_wheel.setTargetPosition(-position);
        front_left_wheel.setTargetPosition(-position);
        back_left_wheel.setTargetPosition(position);
        back_right_wheel.setTargetPosition(position);
        for (DcMotor motor : motors) {
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((front_left_wheel.isBusy() || front_right_wheel.isBusy() || back_right_wheel.isBusy()
                || back_left_wheel.isBusy()) && opModeIsActive()) {
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

    public void goBack(int position) {
        front_right_wheel.setTargetPosition(position);
        front_left_wheel.setTargetPosition(-position);
        back_left_wheel.setTargetPosition(-position);
        back_right_wheel.setTargetPosition(position);
        for (DcMotor motor : motors) {
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((front_left_wheel.isBusy() || front_right_wheel.isBusy() || back_right_wheel.isBusy()
                || back_left_wheel.isBusy()) && opModeIsActive()) {
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

