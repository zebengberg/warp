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
    private BNO055IMU imu;



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
        }

        // Initialing encoders.
        // Forward-reverse encoder.
        front_left_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Side-side encoder
        front_right_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Make sure the imu gyro is calibrated before continuing.
        telemetry.addData("Status", "Calibrating gyro...");
        telemetry.update();
        while (!opModeIsActive() && !imu.isGyroCalibrated())
        {
            idle();
        }


        telemetry.addData("Status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized!");
        telemetry.update();


        telemetry.speak("jens jens jellybeans");

        // wait for start button
        waitForStart();

        while (opModeIsActive()) {
            goForward(50000);
            goBack(0);
            sleep(5000);
            goForward(50000);
            goBack(0);
            sleep(5000);
            goForward(50000);
            goBack(0);
            sleep(5000);
            goForward(50000);
            goBack(0);
            sleep(5000);


        }
    }


//    private void rotateCW (int position){
//        front_right_wheel.setTargetPosition(position);
//        front_left_wheel.setTargetPosition(position);
//        back_left_wheel.setTargetPosition(position);
//        back_right_wheel.setTargetPosition(position);
//        for (DcMotor motor : motors) {
//            motor.setPower(.3);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        while ((busyCounter() >= 2) && opModeIsActive()) {
//            telemetry.addData("target position", position);
//            for (DcMotor motor : motors) {
//                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
//            }
//            telemetry.update();
//        }
//
//        for (DcMotor motor : motors) {
//            motor.setPower(0);
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//    }
//
//    private void rotateCCW ( int position){
//        front_right_wheel.setTargetPosition(-position);
//        front_left_wheel.setTargetPosition(-position);
//        back_left_wheel.setTargetPosition(-position);
//        back_right_wheel.setTargetPosition(-position);
//        for (DcMotor motor : motors) {
//            motor.setPower(.3);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        while ((busyCounter() >= 2) && opModeIsActive()) {
//            telemetry.addData("target position", position);
//            for (DcMotor motor : motors) {
//                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
//            }
//            telemetry.update();
//        }
//
//        for (DcMotor motor : motors) {
//            motor.setPower(0);
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//    }
//
//
//    private void goForward ( int position){
//        front_right_wheel.setTargetPosition(-position);
//        front_left_wheel.setTargetPosition(position);
//        back_left_wheel.setTargetPosition(position);
//        back_right_wheel.setTargetPosition(-position);
//        for (DcMotor motor : motors) {
//            motor.setPower(.3);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        while ((busyCounter() >= 2) && opModeIsActive()) {
//            telemetry.addData("target position", position);
//            for (DcMotor motor : motors) {
//                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
//            }
//            telemetry.update();
//        }
//
//        for (DcMotor motor : motors) {
//            motor.setPower(0);
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//    }
//
//    private void goRight ( int position){
//        front_right_wheel.setTargetPosition(position);
//        front_left_wheel.setTargetPosition(position);
//        back_left_wheel.setTargetPosition(-position);
//        back_right_wheel.setTargetPosition(-position);
//        for (DcMotor motor : motors) {
//            motor.setPower(.3);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        while ((busyCounter() >= 2) && opModeIsActive()) {
//            telemetry.addData("target position", position);
//            for (DcMotor motor : motors) {
//                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
//            }
//            telemetry.update();
//        }
//
//        for (DcMotor motor : motors) {
//            motor.setPower(0);
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//    }
//
//    private void goLeft ( int position){
//        front_right_wheel.setTargetPosition(-position);
//        front_left_wheel.setTargetPosition(-position);
//        back_left_wheel.setTargetPosition(position);
//        back_right_wheel.setTargetPosition(position);
//        for (DcMotor motor : motors) {
//            motor.setPower(.3);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        while ((busyCounter() >= 2) && opModeIsActive()) {
//            telemetry.addData("target position", position);
//            for (DcMotor motor : motors) {
//                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
//            }
//            telemetry.update();
//        }
//
//        for (DcMotor motor : motors) {
//            motor.setPower(0);
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//    }
//
//    private void goBack ( int position){
//        front_right_wheel.setTargetPosition(position);
//        front_left_wheel.setTargetPosition(-position);
//        back_left_wheel.setTargetPosition(-position);
//        back_right_wheel.setTargetPosition(position);
//
//        for (DcMotor motor : motors) {
//            motor.setPower(.3);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        while ((busyCounter() >= 2) && opModeIsActive()) {
//            telemetry.addData("target position", position);
//            for (DcMotor motor : motors) {
//                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
//            }
//            telemetry.update();
//        }
//
//        for (DcMotor motor : motors) {
//            motor.setPower(0);
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//    }
//
//    private int busyCounter() {
//        int busyCount = 0;
//        for (DcMotor motor : motors) {
//            if (motor.isBusy()) {
//                busyCount++;
//            }
//        }
//        return busyCount;
//    }

    private void jens_da_man() {
        goForward(2600);
        wrist.setPosition(0.5);
        sleep(500);
        goBack(400);
        goRight(2000);
        left_lift.setTargetPosition(100);
        right_lift.setTargetPosition(100);
        goForward(400);
        wrist.setPosition(0.0);
        left_lift.setTargetPosition(0);
        right_lift.setTargetPosition(0);
        sleep(500);
        goBack(400);
        goLeft(4500);
        goForward(400);
        wrist.setPosition(0.5);
        sleep(500);
        goBack(400);
        goRight(4500);
        left_lift.setTargetPosition(100);
        right_lift.setTargetPosition(100);
        goForward(400);
        wrist.setPosition(0.0);
        left_lift.setTargetPosition(0);
        right_lift.setTargetPosition(0);
        left_platform.setPosition(0.5);
        right_platform.setPosition(0.5);
        goBack(2400);
        goLeft(1500);
        goForward(400);
        goRight(600);
        goBack(400);
        goLeft(2000);
    }



    private void goForward(int position) {
        // Using front left wheel as a proxy for forward-reverse position

        while((-front_left_wheel.getCurrentPosition() < position) && opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;

            if (gyro < -0.03) {  // drifting CW; adjust by emphasizing CCW rotation
                front_right_wheel.setPower(-1.0);
                front_left_wheel.setPower(0.9);
                back_left_wheel.setPower(0.9);
                back_right_wheel.setPower(-1.0);
            } else if (gyro > 0.03) {  // opposite of above
                front_right_wheel.setPower(-0.9);
                front_left_wheel.setPower(1.0);
                back_left_wheel.setPower(1.0);
                back_right_wheel.setPower(-0.9);
            } else {
                front_right_wheel.setPower(-1);
                front_left_wheel.setPower(1);
                back_left_wheel.setPower(1);
                back_right_wheel.setPower(-1);
            }

            printDebug();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private void goBack(int position) {
        // Using front left wheel as a proxy for forward-reverse position

        while((front_left_wheel.getCurrentPosition() < position) && opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;

            if (gyro < -0.01) {  // drifting CW; adjust by emphasizing CCW rotation
                front_right_wheel.setPower(1.0);
                front_left_wheel.setPower(-0.9);
                back_left_wheel.setPower(-0.9);
                back_right_wheel.setPower(1.0);
            } else if (gyro > 0.01) {  // opposite of above
                front_right_wheel.setPower(0.9);
                front_left_wheel.setPower(-1.0);
                back_left_wheel.setPower(-1.0);
                back_right_wheel.setPower(0.9);
            } else {
                front_right_wheel.setPower(1.0);
                front_left_wheel.setPower(-1.0);
                back_left_wheel.setPower(-1.0);
                back_right_wheel.setPower(-1.0);
            }

            printDebug();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private void goRight(int position) {
        // Using front right wheel as a proxy for side-side position

        while((front_right_wheel.getCurrentPosition() < position) && opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;

            if (gyro < -0.01) {  // drifting CW; adjust by emphasizing CCW rotation
                front_right_wheel.setPower(0.9);
                front_left_wheel.setPower(0.9);
                back_left_wheel.setPower(-1.0);
                back_right_wheel.setPower(-1.0);
            } else if (gyro > 0.01) {  // opposite of above
                front_right_wheel.setPower(1.0);
                front_left_wheel.setPower(1.0);
                back_left_wheel.setPower(-0.9);
                back_right_wheel.setPower(-0.9);
            } else {
                front_right_wheel.setPower(1.0);
                front_left_wheel.setPower(1.0);
                back_left_wheel.setPower(-1.0);
                back_right_wheel.setPower(-1.0);
            }

            printDebug();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private void goLeft(int position) {
        // Using front right wheel as a proxy for side-side position

        while((front_right_wheel.getCurrentPosition() < position) && opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;

            if (gyro < -0.01) {  // drifting CW; adjust by emphasizing CCW rotation
                front_right_wheel.setPower(-0.9);
                front_left_wheel.setPower(-0.9);
                back_left_wheel.setPower(1.0);
                back_right_wheel.setPower(1.0);
            } else if (gyro > 0.01) {  // opposite of above
                front_right_wheel.setPower(-1.0);
                front_left_wheel.setPower(-1.0);
                back_left_wheel.setPower(0.9);
                back_right_wheel.setPower(0.9);
            } else {
                front_right_wheel.setPower(-1.0);
                front_left_wheel.setPower(-1.0);
                back_left_wheel.setPower(1.0);
                back_right_wheel.setPower(1.0);
            }

            printDebug();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }




    private void printDebug() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double gyro = angles.firstAngle;
        telemetry.addData("gyro", gyro);
        telemetry.addData("forward-reverse encoder", front_left_wheel.getCurrentPosition());
        telemetry.addData("side-side encoder", front_right_wheel.getCurrentPosition());
        telemetry.update();
    }
}