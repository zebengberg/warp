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
public class AutoWARPBlocksRedBridgeExperimental extends LinearOpMode {
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



        Servo left_arm = hardwareMap.servo.get("left_arm");
        left_arm.setDirection(Servo.Direction.FORWARD);
        left_arm.setPosition(0.0);

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


        telemetry.speak("jens jens jens jens jellybeans jellybeans jellybeans");

        // wait for start button
        waitForStart();

        while (opModeIsActive()) {
            moveTo(23500, 0);
            left_arm.setPosition(0.85);
            sleep(1000);
            moveTo(17000, 0);
            rotate(Math.PI / 2);
            moveTo(45000, 0);
            left_arm.setPosition(0.0);
            sleep(1000);
            moveTo(17000, 0);
            rotate(Math.PI / 2);
        }
    }



    private void jens_da_man() {


        wrist.setPosition(0.5);
        sleep(500);

        left_lift.setTargetPosition(100);
        right_lift.setTargetPosition(100);

        wrist.setPosition(0.0);
        left_lift.setTargetPosition(0);
        right_lift.setTargetPosition(0);
        sleep(500);

        wrist.setPosition(0.5);
        sleep(500);

        left_lift.setTargetPosition(100);
        right_lift.setTargetPosition(100);

        wrist.setPosition(0.0);
        left_lift.setTargetPosition(0);
        right_lift.setTargetPosition(0);
        left_platform.setPosition(0.5);
        right_platform.setPosition(0.5);

    }



    private void moveTo(int x, int y) {
        // Using front left wheel as a proxy for forward-reverse position
        // Using front right wheel as a proxy for side-side position
        double delta_x = x - front_right_wheel.getCurrentPosition();
        double delta_y = y + front_left_wheel.getCurrentPosition();  // encoder associated to front_left_wheel backwards
        double theta = Math.atan2(delta_y, delta_x);

        // (cos, sin) = ne(1, 1) + nw(-1, 1)
        // Now dot with sides with (1, 1) and (-1, 1), then rescale.
        double ne = Math.cos(theta) + Math.sin(theta);  // component in NE direction
        double nw = -Math.cos(theta) + Math.sin(theta);  // component in NW direction


        while (Math.signum(delta_x) == Math.signum(x - front_right_wheel.getCurrentPosition()) ||
                (Math.signum(delta_y) == Math.signum(y + front_left_wheel.getCurrentPosition()))) {

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double current_gyro = angles.firstAngle;

            double lambda = Math.max(Math.abs(ne), Math.abs(nw)) + Math.abs(current_gyro);
            if (lambda > 1) {
                ne /= lambda;
                nw /= lambda;
            }

            front_left_wheel.setPower(ne + current_gyro);
            front_right_wheel.setPower(-nw + current_gyro);
            back_right_wheel.setPower(-ne + current_gyro);
            back_left_wheel.setPower(nw + current_gyro);

            telemetry.addData("theta", theta);
            telemetry.addData("delta_x", delta_x);
            telemetry.addData("delta_y", delta_y);
            telemetry.addData("current gyro", current_gyro);
            telemetry.update();
        }

        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private void rotate(double theta) {
        if (theta > 0) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;
            while(Math.abs(theta - gyro) > .01) {
                for (DcMotor motor : motors) {
                    // TODO: use some decelerate function
                    motor.setPower(decelAngle(theta - gyro));
                }
            }
        }
    }

    private double decelAngle(double theta) {
        if (Math.abs(theta) > 0.3) {
            return Math.signum(theta);
        } else {
            return theta / 0.3;
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