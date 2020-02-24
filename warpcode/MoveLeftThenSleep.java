package org.firstinspires.ftc.teamcode.warpcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


// WARP Dec 2019

@Autonomous
public class MoveLeftThenSleep extends LinearOpMode {
    private DcMotor front_left_wheel;
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_right_wheel;
    private DcMotor[] motors;
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
        }

        // Initialing encoders.
        // Forward-reverse encoder.
        front_left_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Side-side encoder
        front_right_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU DEVICE
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // wait for start button
        waitForStart();

        if (opModeIsActive()) {
            moveTo(-1000, 1000);
            moveTo(-19000, 1000);
            moveTo(-20000, -1000);
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
            double lambda = Math.max(Math.abs(ne), Math.abs(nw)) + Math.abs(gyro);
            if (lambda < 1) { lambda = 1; }
            ne = ne * decelLinear(dist) / lambda;
            nw = nw * decelLinear(dist) / lambda;

            front_left_wheel.setPower(ne + gyro);
            front_right_wheel.setPower(-nw + gyro);
            back_right_wheel.setPower(-ne + gyro);
            back_left_wheel.setPower(nw + gyro);

            telemetry.addData("theta", theta);
            telemetry.addData("gyro", gyro);
            telemetry.addData("delta_x", delta_x);
            telemetry.addData("delta_y", delta_y);
            telemetry.addData("dist", dist);
            telemetry.addData("ne", ne);
            telemetry.addData("nw", nw);
            telemetry.update();
        } while (((sign_x * (x - get_x()) > 200) || (sign_y * (y - get_y()) > 200)) && opModeIsActive());

        for (DcMotor motor : motors) {
            motor.setPower(0);
        }

    }

    private double decelLinear(double distance) {
        if (distance > 5000) {
            return .3;
        } else {
            return 0.1 * distance / 5000 + .2;
        }
    }

    private int get_x() { return front_right_wheel.getCurrentPosition(); }

    private int get_y() { return -front_left_wheel.getCurrentPosition(); }

}

