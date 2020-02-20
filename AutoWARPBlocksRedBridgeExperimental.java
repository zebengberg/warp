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


@Autonomous
public class AutoWARPBlocksRedBridgeExperimental extends LinearOpMode {
    private DcMotor front_left_wheel;
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_right_wheel;
    private DcMotor[] motors;
    private BNO055IMU imu;

    private double reset_gyro = 0.0;

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


        Servo left_arm = hardwareMap.servo.get("left_arm");
        left_arm.setDirection(Servo.Direction.FORWARD);
        left_arm.setPosition(0.0);

        Servo wrist = hardwareMap.servo.get("wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(0.0);


        // Motors for big arm.
//        DcMotor left_lift = hardwareMap.dcMotor.get("left_lift");
//        DcMotor right_lift = hardwareMap.dcMotor.get("right_lift");
//        left_lift.setDirection(DcMotor.Direction.FORWARD);
//        right_lift.setDirection(DcMotor.Direction.FORWARD);
//        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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
        while (!opModeIsActive() && !imu.isGyroCalibrated()) { idle(); }
        telemetry.addData("Status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized!");
        telemetry.update();


        telemetry.speak("jens jens jens jens jellybeans");

        // wait for start button
        waitForStart();

        while (opModeIsActive()) {
            moveTo(0, -24000);
            left_arm.setPosition(0.85);
            sleep(1000);
            moveTo(0, -17000);
            rotateTo(Math.PI / 2);


            moveTo(0, -60000);
            left_arm.setPosition(0.0);
            sleep(1000);
            moveTo(0, 0);
            rotateTo(0);
            moveTo(-2000, -6000);

            left_arm.setPosition(0.85);
            sleep(1000);
            moveTo(0, 0);
            rotateTo(Math.PI / 2);
            moveTo(0, -60000);

            rotateTo(100);


            //left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (left_lift.isBusy() && right_lift.isBusy()) {
//               left_lift.setPower(1.0);
//                right_lift.setPower(1.0);
//                telemetry.addData("left lift", left_lift.getCurrentPosition());
//                telemetry.addData("right lift", right_lift.getCurrentPosition());
//                telemetry.update();
//            }





//            sleep(2000);
//            moveTo(55000, 26000);
//            wrist.setPosition(0);
//            moveTo(52443, 13000);
//
//            moveTo(-18139, -12950);
//            moveTo(-18139, -23700);
//            wrist.setPosition(1.0);
//            moveTo(-18139, 12950);
//            moveTo(62200, -12950);
//
//            moveTo(62200, -23700 );
//            wrist.setPosition(0);
//            sleep(500);
////            left_platform.setPosition(1.0);
////            right_platform.setPosition(1.0);
//            sleep(500);
//            moveTo(62200, 0);
////            left_platform.setPosition(0.0);
////            right_platform.setPosition(0.0);
//            moveTo(0, 39200);
//            moveTo(39200, -13264);
//            moveTo(43427,-13264 );
//            moveTo(43200, -13264);
//            moveTo(43427, -22183);
//            moveTo(21800, -22183);
        }
    }




    private void moveTo(int x, int y) {
        // Using front left wheel as a proxy for forward-reverse position
        // Using front right wheel as a proxy for side-side position

        int sign_x = Integer.signum(x - get_x());
        int sign_y = Integer.signum(y - get_y());


        do {
            double delta_x = x - get_x();
            double delta_y = y - get_y();
            double theta = Math.atan2(delta_y, delta_x);
            double dist = Math.floor(Math.sqrt(delta_x * delta_x + delta_y * delta_y));

            // (cos, sin) = ne(1, 1) + nw(-1, 1)
            // Now dot with sides with (1, 1) and (-1, 1), then rescale.
            double ne = Math.cos(theta) + Math.sin(theta);  // component in NE direction
            double nw = -Math.cos(theta) + Math.sin(theta);  // component in NW direction
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;
            double lambda = Math.max(Math.abs(ne), Math.abs(nw)) + Math.abs(gyro - reset_gyro);
            if (lambda < 1) { lambda = 1; }
            ne = ne * decelLinear(dist) / lambda;
            nw = nw * decelLinear(dist) / lambda;

            front_left_wheel.setPower(ne + gyro - reset_gyro);
            front_right_wheel.setPower(-nw + gyro - reset_gyro);
            back_right_wheel.setPower(-ne + gyro - reset_gyro);
            back_left_wheel.setPower(nw + gyro - reset_gyro);

            telemetry.addData("theta", theta);
            telemetry.addData("gyro", gyro);
            telemetry.addData("reset gyro", reset_gyro);
            telemetry.addData("delta_x", delta_x);
            telemetry.addData("delta_y", delta_y);
            telemetry.addData("dist", dist);
            telemetry.addData("ne", ne);
            telemetry.addData("nw", nw);
            telemetry.update();
        } while ((sign_x * (x - get_x()) > 100) || (sign_y * (y - get_y()) > 100));

        for (DcMotor motor : motors) {
            motor.setPower(0);
        }

    }

    private double decelLinear(double distance) {
        if (distance > 10000) {
            return .8;
        } else {
            return .8 * distance / 10000 + .2;
        }
    }

    private void rotateTo(double theta) {
        // TODO: theta < 0

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double gyro = angles.firstAngle;
        while(Math.abs(theta + gyro) > .1) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            gyro = angles.firstAngle;
            for (DcMotor motor : motors) {
                motor.setPower(decelRotation(theta + gyro));
            }
            telemetry.addData("theta", theta);
            telemetry.addData("gyro", gyro);
            telemetry.update();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
        reset_gyro = gyro;
    }

    private double decelRotation(double theta) {
        if (Math.abs(theta) > 0.5) {
            return .8;
            //return Math.signum(theta);
        } else {
            return .8 * theta / 0.5;
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

    private int get_x() { return front_right_wheel.getCurrentPosition(); }

    private int get_y() { return -front_left_wheel.getCurrentPosition(); }

    private boolean isYellow(ColorSensor sensor) {
        float[] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(sensor.red() * 255, sensor.green() * 255, sensor.blue() * 255, hsv);
        float hue = hsv[0];  // sat = hsv[1] and val = hsv[2]
        return (hue < 90);
    }
}