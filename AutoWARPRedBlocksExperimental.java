package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


// WARP Dec 2019

@Autonomous
public class AutoWARPRedBlocksExperimental extends LinearOpMode {
    private DcMotor front_left_wheel;
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_right_wheel;
    private DcMotor[] motors;

    private Servo right_arm;
    private Servo left_arm;
    private ColorSensor right_color;

    // Creating a macro for speed of DC motors.
    private double speed = 0.3;

    // IMU DEVICE
    private BNO055IMU imu;




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
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Servos for little arms
        left_arm = hardwareMap.servo.get("left_arm");
        right_arm = hardwareMap.servo.get("right_arm");
        left_arm.setDirection(Servo.Direction.FORWARD);
        right_arm.setDirection(Servo.Direction.REVERSE);
        left_arm.setPosition(0);
        right_arm.setPosition(0);

        // Sensors
        right_color = hardwareMap.get(ColorSensor.class, "right_color");


        // IMU DEVICE
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Try to calibrate imu before op mode starts.
        telemetry.addData("Status", "calibrating gyro");
        telemetry.update();
        while (!opModeIsActive() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "gyro calibrated");
        for (DcMotor motor : motors) {
            telemetry.addData("motor mode", motor.getMode());
        }
        telemetry.update();



        int block_count = 0;
        int block_size = 710;

        // wait for start button
        waitForStart();
        // Going to the blocks initially. Finding the first black block.
        if (opModeIsActive()) {
            goLeft(10000);
            goLeft(2240);
            while (true) {
                if (isYellow() & block_count < 2) {
                    goForward(block_size);
                    block_count++;
                } else {
                    block_count++;
                    break;
                }
            }

            // Grabbing the black block and moving back toward fence.
            right_arm.setPosition(0.70);
            sleep(1000);
            goRight(800);

            // Going under the skystone bridge. Releasing block. Moving back toward fence a hair.
            goBack(2250 + block_count * block_size);
            right_arm.setPosition(0);
            sleep(500);
            goRight(200);
            block_count += 3;

            // Going back to the blocks. Finding a black one.
            goForward(1750 + block_count * block_size);
            goLeft(1150);
            while (true) {
                if (isYellow()) {
                    goForward(200);
                } else {
                    goForward(200);
                    break;
                }
            }

            // Grabbing the black block and moving back toward fence.
            right_arm.setPosition(0.70);
            sleep(1000);
            goRight(600);

            // Going under the skystone bridge and releasing block.
            goBack(2000 + block_count * block_size);
            right_arm.setPosition(0);
            sleep(500);
            goRight(200);

            // Going to sky bridge tape.
            goForward(1100);

            // Doing some stuff to get ready for autonomous
            goLeft(1000);
            right_arm.setPosition(0.4);
            left_arm.setPosition(0.4);
        }
    }

    private boolean isYellow() {
        float[] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(right_color.red() * 255, right_color.green() * 255, right_color.blue() * 255, hsv);
        float hue = hsv[0];  // sat = hsv[1] and val = hsv[2]
        return (hue < 90);
    }



    private void goForward(int position) {
        while (((front_left_wheel.getCurrentPosition() +
                back_left_wheel.getCurrentPosition() +
                front_right_wheel.getCurrentPosition() +
                back_right_wheel.getCurrentPosition()) / 4  < position) && opModeIsActive()) {
            // Change the power if the robot is not moving in the correct direction.
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;

            // (cos, sin) = ne(1, 1) + nw(-1, 1)
            // Now dot with sides with (1, 1) and (-1, 1), then rescale.
            double ne = Math.cos(gyro) + Math.sin(gyro);  // component in NE direction
            double nw = -Math.cos(gyro) + Math.sin(gyro);  // component in NW direction
            ne *= speed;  //scale
            nw *= speed;

            front_left_wheel.setPower(ne);
            front_right_wheel.setPower(-nw);
            back_right_wheel.setPower(-ne);
            back_left_wheel.setPower(nw);
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void goRight(int position) {
        while (((front_left_wheel.getCurrentPosition() -
                back_left_wheel.getCurrentPosition() +
                front_right_wheel.getCurrentPosition() -
                back_right_wheel.getCurrentPosition()) / 4  < position) && opModeIsActive()) {
            // Change the power if the robot is not moving in the correct direction.
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;

            // (cos, sin) = ne(1, 1) + nw(-1, 1)
            // Now dot with sides with (1, 1) and (-1, 1), then rescale.
            double ne = Math.cos(gyro) + Math.sin(gyro);  // component in NE direction
            double nw = -Math.cos(gyro) + Math.sin(gyro);  // component in NW direction
            ne *= speed;  //scale
            nw *= speed;

            front_left_wheel.setPower(ne);
            front_right_wheel.setPower(nw);
            back_right_wheel.setPower(-ne);
            back_left_wheel.setPower(-nw);
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    private void goLeft(int position) {
        while (((front_left_wheel.getCurrentPosition() +
                back_left_wheel.getCurrentPosition() +
                front_right_wheel.getCurrentPosition() +
                back_right_wheel.getCurrentPosition()) / 4  < position) && opModeIsActive()) {

            // Change the power if the robot is not moving in the correct direction.
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;

            // (cos, sin) = ne(1, 1) + nw(-1, 1)
            // Now dot with sides with (1, 1) and (-1, 1), then rescale.
            double ne = Math.cos(gyro) + Math.sin(gyro);  // component in NE direction
            double nw = -Math.cos(gyro) + Math.sin(gyro);  // component in NW direction
            ne *= speed;  //scale
            nw *= speed;

            front_left_wheel.setPower(-ne);
            front_right_wheel.setPower(-nw);
            back_right_wheel.setPower(ne);
            back_left_wheel.setPower(nw);

            telemetry.addData("average position", (-front_left_wheel.getCurrentPosition() +
                    back_left_wheel.getCurrentPosition() -
                    front_right_wheel.getCurrentPosition() +
                    back_right_wheel.getCurrentPosition()) / 4);
            telemetry.addData("front left power", front_left_wheel.getPower());
            telemetry.addData("front right power", front_right_wheel.getPower());
            telemetry.addData("back left power", back_left_wheel.getPower());
            telemetry.addData("back right power", back_right_wheel.getPower());
            telemetry.addData("gyro", gyro);
            telemetry.update();

        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }



    private void goBack(int position) {
        while ((-(front_left_wheel.getCurrentPosition() -
                back_left_wheel.getCurrentPosition() +
                front_right_wheel.getCurrentPosition() +
                back_right_wheel.getCurrentPosition()) / 4  < position) && opModeIsActive()) {
            // Change the power if the robot is not moving in the correct direction.
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;

            // (cos, sin) = ne(1, 1) + nw(-1, 1)
            // Now dot with sides with (1, 1) and (-1, 1), then rescale.
            double ne = Math.cos(gyro) + Math.sin(gyro);  // component in NE direction
            double nw = -Math.cos(gyro) + Math.sin(gyro);  // component in NW direction
            ne *= speed;  //scale
            nw *= speed;

            front_left_wheel.setPower(-ne);
            front_right_wheel.setPower(nw);
            back_right_wheel.setPower(ne);
            back_left_wheel.setPower(-nw);
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}