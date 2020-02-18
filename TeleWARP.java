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

    private boolean is_slow_pressed = false;
    private boolean slow_state = false;
    private double dc_power_state = 1.0;
    private double rot_power_state = 0.6;
    private double target_angle = 0.0;

    private Servo left_arm;
    private Servo right_arm;

    private Servo left_platform;
    private Servo right_platform;
    private boolean is_platform_pressed = false;
    private boolean platform_state = false;

    private Servo wrist;
    private boolean is_wrist_pressed = false;
    private boolean wrist_state = false;

    private DcMotor left_lift;
    private DcMotor right_lift;


    // The lift has four different states
    // State 0: all the way down in starting position
    // State 1: at the apex above the block to be stacked
    // State 2: down on top of the next block
    // State 3: back up at the apex after block has been released
    private boolean is_lift_pressed = false;
    private int lift_state = 0;
    private int lift_block_number = 0;
    private int[] lift_targets = {100, 500, 900, 1300, 1700, 2100};

    private BNO055IMU imu;





    @Override
    public void runOpMode() {
        // DC motors for holonomic drive.
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");

        // Creating an array of motors so we can iterate over it.
        DcMotor[] motors = {back_left_wheel, back_right_wheel, front_right_wheel, front_left_wheel};

        // Initializing the motors.
        for (DcMotor motor : motors) {
            // REV HD Hex encoder counts 2240 per rotation.
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double reset_angle = 0.0;

        // Servos for little arms
        left_arm = hardwareMap.servo.get("left_arm");
        right_arm = hardwareMap.servo.get("right_arm");
        left_arm.setDirection(Servo.Direction.FORWARD);
        right_arm.setDirection(Servo.Direction.REVERSE);
        left_arm.setPosition(0.0);
        right_arm.setPosition(0.0);



        // Servos for moving the platform.
        left_platform = hardwareMap.servo.get("left_platform");
        right_platform = hardwareMap.servo.get("right_platform");
        left_platform.setDirection(Servo.Direction.REVERSE);
        right_platform.setDirection(Servo.Direction.FORWARD);
        left_platform.setPosition(0.0);
        right_platform.setPosition(0.0);


        // Motors for big arm.
        left_lift = hardwareMap.dcMotor.get("left_lift");
        right_lift = hardwareMap.dcMotor.get("right_lift");
        left_lift.setDirection(DcMotor.Direction.FORWARD);
        right_lift.setDirection(DcMotor.Direction.FORWARD);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        wrist = hardwareMap.servo.get("wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(0.0);





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
        while (!isStarted() && !imu.isGyroCalibrated()) { idle(); }

        telemetry.addData("Status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // wait for start button
        telemetry.speak("jen jen jen jellybean");
        waitForStart();

        while (opModeIsActive()) {
            //printStatus();


            //TODO: deal with rotation by more than 180.
            // Dealing with rotation stuff; all rotation controlled by right_stick.
            target_angle += 0.1 * gamepad1.right_stick_x * rot_power_state;
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;
            if (gamepad1.y) {
                reset_angle = gyro;
            }
            double rotate = target_angle + gyro + reset_angle;


            // Dealing with velocity vector; all velocity controlled by left_stick.
            double x = gamepad1.left_stick_x * dc_power_state;
            double y = -gamepad1.left_stick_y * dc_power_state;
            double theta = Math.atan2(y, x);
            double r = Math.sqrt(x * x + y * y);

            // (cos, sin) = ne(1, 1) + nw(-1, 1)
            // Now dot with sides with (1, 1) and (-1, 1), then rescale.
            double ne = r * (Math.cos(theta) + Math.sin(theta));  // component in NE direction
            double nw = r * (-Math.cos(theta) + Math.sin(theta));  // component in NW direction

            // may still need to scale to adjust for rotate
            double lambda = Math.max(Math.abs(ne), Math.abs(nw)) + Math.abs(rotate);
            if (lambda > 1) {
                ne /= lambda;
                nw /= lambda;
            }

            telemetry.addData("theta", theta);
            telemetry.addData("ne", ne);
            telemetry.addData("nw", nw);
            telemetry.addData("rotate", rotate);
            telemetry.update();

            front_left_wheel.setPower(ne + rotate);
            front_right_wheel.setPower(-nw + rotate);
            back_right_wheel.setPower(-ne + rotate);
            back_left_wheel.setPower(nw + rotate);



            // Small arms
            if (gamepad1.left_bumper) {
                left_arm.setPosition(0.85);
            } else {
                left_arm.setPosition(0.0);
            }
            if (gamepad1.right_bumper) {
                right_arm.setPosition(0.75);
            } else {
                right_arm.setPosition(0.0);
            }

            // Toggle platform
            if (gamepad1.b && !is_platform_pressed) {
                is_platform_pressed = true;
                togglePlatformGrabbers();
            } else if (!gamepad1.b) {
                is_platform_pressed = false;
            }

            // Toggle wrist
            if (gamepad1.a && !is_wrist_pressed) {
                is_wrist_pressed = true;
                toggleWrist();
            } else if (!gamepad1.a) {
                is_wrist_pressed = false;
            }

            // Toggle slow
            if (gamepad1.x && !is_slow_pressed) {
                is_slow_pressed = true;
                toggleSlow();
            } else if (!gamepad1.x) {
                is_slow_pressed = false;
            }

            // Controlling the lift manually.
            if (gamepad1.right_trigger > 0) {
                left_lift.setPower(gamepad1.right_trigger);
                right_lift.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                left_lift.setPower(-gamepad1.left_trigger);
                right_lift.setPower(-gamepad1.left_trigger);
            } else {
                left_lift.setPower(0);
                right_lift.setPower(0);
            }

            // Controlling the lift automatically.
            if (gamepad1.dpad_up && !is_lift_pressed) {
                is_lift_pressed = true;
                automateLift();
            }else if (!gamepad1.dpad_up) {
                is_lift_pressed = false;
            }
        }
    }


    private void printStatus() {
        telemetry.addData("Front Left ", front_left_wheel.getPower());
        telemetry.addData("Front Right", front_right_wheel.getPower());
        telemetry.addData("Back Left  ", back_left_wheel.getPower());
        telemetry.addData("Back Right ", back_right_wheel.getPower());

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double gyro = angles.firstAngle;
        telemetry.addData("gyro angle", gyro);
        telemetry.addData("target angle", target_angle);

        telemetry.addData("Left Arm ", left_arm.getPosition());
        telemetry.addData("Right Arm", right_arm.getPosition());

        telemetry.addData("Left lift position", left_lift.getCurrentPosition());
        telemetry.addData("Right lift position", right_lift.getCurrentPosition());

        telemetry.addData("wrist position", wrist.getPosition());

        telemetry.addData("left platform", left_platform.getPosition());
        telemetry.addData("right platform", right_platform.getPosition());

        telemetry.addData("lift block number", lift_block_number);

        telemetry.update();
    }



    private void automateLift() {
        left_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        lift_state++;
        lift_state %= 4;
        int height = lift_targets[lift_block_number];
        switch (lift_state) {
            case 0:
                left_lift.setTargetPosition(height);
                right_lift.setTargetPosition(height);
                break;
            case 1:
                left_lift.setTargetPosition(height - 50);
                right_lift.setTargetPosition(height - 50);
                break;
            case 2:
                left_lift.setTargetPosition(height);
                right_lift.setTargetPosition(height);
                break;
            case 3:
                left_lift.setTargetPosition(0);
                right_lift.setTargetPosition(0);
                lift_block_number++;
                break;
        }

        while (left_lift.isBusy() && right_lift.isBusy()) {
            idle();
        }
    }

    private void togglePlatformGrabbers() {
        platform_state = !platform_state;
        if (platform_state) {
            left_platform.setPosition(0.3);
            right_platform.setPosition(0.3);
        } else {
            left_platform.setPosition(0);
            right_platform.setPosition(0);
        }
    }

    private void toggleWrist() {
        wrist_state = !wrist_state;
        if (wrist_state) {
            wrist.setPosition(1.0);
        } else {
            wrist.setPosition(0);
        }
    }

    private void toggleSlow() {
        slow_state = !slow_state;
        if (slow_state) {
            dc_power_state = 0.3;
            rot_power_state = 0.3;
        } else {
            dc_power_state = 1.0;
            rot_power_state = 0.6;
        }
    }
}


