package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



// HOLONOMIC GYRO DRIVE.
// Includes imu gyro correction, using joystick to pass velocity vector, and simultaneous rotation.
// WARP Nov 2019
@TeleOp
public class TeleWARP extends LinearOpMode {

    DcMotor front_left_wheel;
    DcMotor back_left_wheel;
    DcMotor back_right_wheel;
    DcMotor front_right_wheel;

    Servo left_arm;
    Servo right_arm;

    BNO055IMU imu;

    DcMotor big_arm_rotate;
    DcMotor big_arm_extend;
    Servo big_arm_thumb;





    @Override
    public void runOpMode() {
        // DC motors for holonomic drive.
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");
        // Arbitrary decision to reverse all -- could instead negate powers below.
        front_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        front_right_wheel.setDirection(DcMotor.Direction.REVERSE);
        back_right_wheel.setDirection(DcMotor.Direction.REVERSE);
        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos for little arms
        left_arm = hardwareMap.servo.get("left_arm");
        right_arm = hardwareMap.servo.get("right_arm");
        left_arm.resetDeviceConfigurationForOpMode();
        right_arm.resetDeviceConfigurationForOpMode();
        left_arm.setDirection(Servo.Direction.FORWARD);
        right_arm.setDirection(Servo.Direction.REVERSE);

        // Motors for big arm
        big_arm_rotate = hardwareMap.dcMotor.get("big_arm_rotate");
        big_arm_extend = hardwareMap.dcMotor.get("big_arm_extend");
        big_arm_thumb = hardwareMap.servo.get("big_arm_thumb");
        big_arm_thumb.resetDeviceConfigurationForOpMode();

        // IMU DEVICE -- possibly delete some of this.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Make sure the imu gyro is calibrated before continuing.
        telemetry.addData("Status", "calibrating gyro");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(10);
            idle();
        }

        telemetry.addData("Status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // wait for start button
        waitForStart();

        while (opModeIsActive()) {
            // Controlling holonomic drive.
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;  // for some reason y-component opposite of Descartes
            double theta = Math.atan2(y, x);
            double r = Math.sqrt(x*x + y*y);

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;
            double angle = theta - gyro;  // for some reason gyro is opposite of mathworld
            double rotate = gamepad1.right_stick_x/3;  // rescale to slow it down.

            // (cos, sin) = ne(1, 1) + nw(-1, 1)
            // Now dot with sides with (1, 1) and (-1, 1), then rescale.
            double ne = Math.cos(angle) + Math.sin(angle);  // component in NE direction
            double nw = -Math.cos(angle) + Math.sin(angle);  // component in NW direction
            ne *= r;  //scale
            nw *= r;
            // may still need to scale to adjust for rotate
            double lambda = Math.max(Math.abs(ne), Math.abs(nw)) + Math.abs(rotate);
            if (lambda > 1) {
                ne /= lambda;
                nw /= lambda;
            }

            front_left_wheel.setPower(ne + rotate);
            front_right_wheel.setPower(-nw + rotate);
            back_right_wheel.setPower(-ne + rotate);
            back_left_wheel.setPower(nw + rotate);


            // Little arms
            if (gamepad1.left_bumper) {
                left_arm.setPosition(0.5);
            } else {
                left_arm.setPosition(0);
            }
            if (gamepad1.right_bumper) {
                right_arm.setPosition(0.5);
            } else {
                right_arm.setPosition(0);
            }


            // Big arm
            if (gamepad1.dpad_down) {
                big_arm_rotate.setPower(.2);
            } else if (gamepad1.dpad_up) {
                big_arm_rotate.setPower(-.2);
            } else {
                big_arm_rotate.setPower(0);
            }

            if (gamepad1.dpad_left) {
                big_arm_extend.setPower(.2);
            } else if (gamepad1.dpad_right) {
                big_arm_extend.setPower(-.2);
            } else {
                big_arm_extend.setPower(0);
            }

            if (gamepad1.a) {
                big_arm_thumb.setPosition(1);
            }


            telemetry.addData("Front Left", front_left_wheel.getPower());
            telemetry.addData("Front Right", front_right_wheel.getPower());
            telemetry.addData("Back Left", back_left_wheel.getPower());
            telemetry.addData("Back Right", back_right_wheel.getPower());
            telemetry.addData("Rotation Angle", gyro * 180 / Math.PI); // degrees
            telemetry.addData("Left Arm", left_arm.getPosition());
            telemetry.addData("Right Arm", right_arm.getPosition());
            telemetry.update();
        }
    }
}


