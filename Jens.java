package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous
public class Jens extends LinearOpMode {
    private DcMotor front_left_wheel;
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_right_wheel;
    private DcMotor left_lift;
    private DcMotor right_lift;
    private Servo left_platform;
    private Servo right_platform;
    private Servo wrist;
    private DcMotor[] motors;
    BNO055IMU imu;



    public void runOpMode() {
        // DC motors for holonomic drive.
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");

        // Creating an array of motors so we can iterate over it.
        motors = new DcMotor[]{back_left_wheel, back_right_wheel, front_right_wheel, front_left_wheel, left_lift, right_lift};

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

        wrist = hardwareMap.servo.get("wrist");

        // IMU DEVICE
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = false;
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Make sure the imu gyro is calibrated before continuing.
        telemetry.addData("Status", "calibrating gyro");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        // wait for start button
        waitForStart();

        if (opModeIsActive()) {



        }


    }
    private void rotateCW ( int position){
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

    private void rotateCCW ( int position){
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


    private void goForward ( int position){
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

    private void goRight ( int position){
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

    private void goLeft ( int position){
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

    private void goBack ( int position){
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

    private void throwonplatform() {
        goForward(2400);
        wrist.setPosition(0.5);
        sleep(500);
        left_lift.setTargetPosition(100);
        right_lift.setTargetPosition(100);
        goBack(400);
        goRight(4000);
        goForward(400);
        wrist.setPosition(0.0);
        sleep(500);
        goBack(400);
        goLeft(4500);
        goForward(400);
        wrist.setPosition(0.5);
        sleep(500);
        left_lift.setTargetPosition(100);
        right_lift.setTargetPosition(100);
        goBack(400);
        goRight(4500);
        goForward(400);
        wrist.setPosition(0.0);
        left_platform.setPosition(0.5);
        right_platform.setPosition(0.5);
        goBack(2400);
        goLeft(1500);
        goForward(400);
        goRight(600);
        goBack(400);
        goLeft(2000);
    }



    private void goStraightForward(int position) {
        // Using front left wheel as a proxy for forward position

        while(front_left_wheel.getCurrentPosition() < position) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;

            if (gyro < -0.01) {  // drifting CCW; adjust by emphasizing CW rotation
                front_right_wheel.setPower(-0.99);
                front_left_wheel.setPower(1);
                back_left_wheel.setPower(1);
                back_right_wheel.setPower(-0.99);
            } else if (gyro > 0.01) {  // opposite of above
                front_right_wheel.setPower(-1);
                front_left_wheel.setPower(0.99);
                back_left_wheel.setPower(0.99);
                back_right_wheel.setPower(-1);
            } else {
                front_right_wheel.setPower(-1);
                front_left_wheel.setPower(1);
                back_left_wheel.setPower(1);
                back_right_wheel.setPower(-1);
            }
        }

        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private void goStraightRight(int position) {
        // Using front left wheel as a proxy for forward position

        while(front_left_wheel.getCurrentPosition() < position) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;

            if (gyro < -0.01) {  // drifting CCW; adjust by emphasizing CW rotation
                front_right_wheel.setPower(-0.99);
                front_left_wheel.setPower(1);
                back_left_wheel.setPower(1);
                back_right_wheel.setPower(-0.99);
            } else if (gyro > 0.01) {  // opposite of above
                front_right_wheel.setPower(-1);
                front_left_wheel.setPower(0.99);
                back_left_wheel.setPower(0.99);
                back_right_wheel.setPower(-1);
            } else {
                front_right_wheel.setPower(-1);
                front_left_wheel.setPower(1);
                back_left_wheel.setPower(1);
                back_right_wheel.setPower(-1);
            }
        }

        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

}