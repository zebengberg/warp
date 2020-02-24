package org.firstinspires.ftc.teamcode.secondarycode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// TeleOp for trianglebot

@TeleOp
public class TriangleBotDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        // DC Motors
        DcMotor motor1 = hardwareMap.dcMotor.get("motor1");  // top
        DcMotor motor2 = hardwareMap.dcMotor.get("motor2");  // bottom right
        DcMotor motor3 = hardwareMap.dcMotor.get("motor3");  // bottom left
        DcMotor[] motors = {motor1, motor2, motor3};
        for (DcMotor motor : motors) {
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // IMU DEVICE
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double gyro = angles.firstAngle;

            double user_rot = gamepad1.right_stick_x / 2;  // scaling to slow down
            double user_y = -gamepad1.left_stick_y;
            double user_x = gamepad1.left_stick_x;

            double theta = Math.atan2(user_y, user_x) - gyro;  // delete gyro from this line to steer relative to robot "front"
            double r = Math.sqrt(user_x * user_x + user_y * user_y);
            double dx = r * Math.cos(theta);
            double dy = r * Math.sin(theta);


            // Let m1, m2, m3 be the power delivered to motors motor1, motor2, motor3. We think of
            // m1, m2, m3 as three unknowns. We have three "constraints": user_rot, dx, and dy.
            // Putting all of these together, we arrive at a system of linear equations.
            // Velocity vector equation:
            // (dx, dy) = m1 * (1, 0) + m2 * (-1/2, -sqrt(3)/2) + m3 * (1/2, -sqrt(3)/2)
            // Angular velocity equation:
            // user_rot = m1 + m2 + m3;
            //
            // Putting this into a matrix form, we have
            // [1        1         1    ] [m1]   [user_rot]
            // [1      -1/2      -1/2   ] [m2] = [dx]
            // [0    -sqrt3/2   sqrt3/2 ] [m3]   [dy]
            // The inverse matrix is
            // [1/3     2/3         0     ]
            // [1/3    -1/3    -sqrt(3)/3 ]
            // [1/3    -1/3     sqrt(3)/3 ]

            double m1 = 1.0/3.0 * user_rot + 2.0/3.0 * dx;
            double m2 = 1.0/3.0 * user_rot - 1.0/3.0 * dx - Math.sqrt(3)/3.0 * dy;
            double m3 = 1.0/3.0 * user_rot - 1.0/3.0 * dx + Math.sqrt(3)/3.0 * dy;

            // May still need to scale to prevent clipping
            double lambda = Math.max(Math.max(Math.abs(m1), Math.abs(m2)), Math.abs(m3));
            if (lambda > 1) {
                m1 /= lambda;
                m2 /= lambda;
                m3 /= lambda;
            }

            motor1.setPower(m1);
            motor2.setPower(m2);
            motor3.setPower(m3);

            telemetry.addData("motor1", motor1.getPower());
            telemetry.addData("motor2", motor2.getPower());
            telemetry.addData("motor3", motor3.getPower());
            telemetry.update();

        }
    }
}
