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
    private DcMotor front_left_wheel;
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_right_wheel;
    private DcMotor[] motors;
    private double reset_angle;
    private double dc_power_state;
    private double dc_power_time;
    private double rot_power_state;
    private double rot_power_time;


    private Servo left_arm;
    private Servo right_arm;
    private boolean small_arms_down_state;
    private double small_arms_down_time;

    private Servo left_platform;
    private Servo right_platform;
    private boolean platform_state;
    private double platform_time;

    private DcMotor big_arm;
    private Servo wrist;
    private boolean wrist_state;
    private double wrist_time;


    BNO055IMU imu;


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
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        reset_angle = 0;
        dc_power_state = 1.0;
        dc_power_time = 0.0;
        rot_power_state = 0.4;
        rot_power_time = 0.0;

        // Servos for little arms
        left_arm = hardwareMap.servo.get("left_arm");
        right_arm = hardwareMap.servo.get("right_arm");
        left_arm.setDirection(Servo.Direction.FORWARD);
        right_arm.setDirection(Servo.Direction.REVERSE);
        left_arm.setPosition(0.45);
        right_arm.setPosition(0.5);
        small_arms_down_state = false;
        small_arms_down_time = 0.0;

        // Servos for moving the platform.
        left_platform = hardwareMap.servo.get("left_platform");
        right_platform = hardwareMap.servo.get("right_platform");
        left_platform.setDirection(Servo.Direction.FORWARD);
        right_platform.setDirection(Servo.Direction.REVERSE);
        left_platform.setPosition(0);
        right_platform.setPosition(0);
        // Keep track of the whether the platform servos are up or down.
        platform_state = false;
        platform_time = 0.0;

        // Motors for big arm.
        big_arm = hardwareMap.dcMotor.get("big_arm");
        wrist = hardwareMap.servo.get("wrist");
        wrist_state = false;
        wrist_time = 0.0;



        // IMU DEVICE
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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

        while (opModeIsActive()) {
            // Controlling holonomic drive.
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;  // for some reason y-component opposite of Descartes

            if (gamepad1.left_stick_button && (dc_power_time < time - 0.5)) {
                if (dc_power_state == 1.0) {
                    dc_power_state = 0.2;
                } else {
                    dc_power_state = 1.0;
                }
            }

            if (gamepad1.right_stick_button && (dc_power_time < time - 0.5)) {
                if (rot_power_state == 0.4) {
                    dc_power_state = 0.1;
                } else {
                    dc_power_state = 0.4;
                }
            }


            double theta = Math.atan2(y, x);
            double r = Math.sqrt(x*x + y*y);

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;
            if (gamepad1.y) { reset_angle = gyro; }
            double angle = theta - gyro + reset_angle;  // for some reason gyro is opposite of math

            double rotate = gamepad1.right_stick_x * rot_power_state;  // rescale to slow it down.

            // (cos, sin) = ne(1, 1) + nw(-1, 1)
            // Now dot with sides with (1, 1) and (-1, 1), then rescale.
            double ne = Math.cos(angle) + Math.sin(angle);  // component in NE direction
            double nw = -Math.cos(angle) + Math.sin(angle);  // component in NW direction
            ne *= r * dc_power_state;  //scale
            nw *= r * dc_power_state;
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
            if (gamepad1.x && (small_arms_down_time < time - 0.5)) {
                small_arms_down_state = !small_arms_down_state;
                small_arms_down_time = time;
            }

            if (gamepad1.left_bumper) {
                left_arm.setPosition(0.69);
            } else {
                if (small_arms_down_state) {
                    left_arm.setPosition(0);
                } else {
                    left_arm.setPosition(0.45);
                }
            }
            if (gamepad1.right_bumper) {
                right_arm.setPosition(0.75);
            } else {
                if (small_arms_down_state) {
                    right_arm.setPosition(0);
                } else {
                    right_arm.setPosition(0.5);
                }
            }


            // Set a delay so that the platform state cannot be changed more often than every 0.5s.
            if (gamepad1.b && (platform_time < time - 0.5)) {
                platform_state = !platform_state;
                platform_time = time;
            }

            if (platform_state) {
                left_platform.setPosition(0.4);
                right_platform.setPosition(0.5);
            } else {
                left_platform.setPosition(0);
                right_platform.setPosition(0);
            }


            // Controlling the big arm.
            if (gamepad1.right_trigger > 0) {
                big_arm.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                big_arm.setPower(- gamepad1.left_trigger);
            } else {
                big_arm.setPower(0);
            }
            // Using a delay to set the wrist grabber.
            if (gamepad1.a && (wrist_time < time - 0.5)) {
                wrist_state = !wrist_state;
                wrist_time = time;
            }

            if (wrist_state) {
                wrist.setPosition(0.6);
            } else {
                wrist.setPosition(0);
            }

            telemetry.addData("Front Left ", front_left_wheel.getPower());
            telemetry.addData("Front Right", front_right_wheel.getPower());
            telemetry.addData("Back Left  ", back_left_wheel.getPower());
            telemetry.addData("Back Right ", back_right_wheel.getPower());
            telemetry.addData("Rotation Angle", gyro * 180 / Math.PI); // degrees
            telemetry.addData("Left Arm ", left_arm.getPosition());
            telemetry.addData("Right Arm", right_arm.getPosition());
            telemetry.addData("Big Arm", big_arm.getPower());
            telemetry.addData("Wrist", wrist.getPosition());
            telemetry.update();
        }
    }
}


