package org.firstinspires.ftc.teamcode.secondarycode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



// TeleOp for minibot.

@TeleOp
public class MiniBotDrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor left_motor = hardwareMap.dcMotor.get("left_motor");
        DcMotor right_motor = hardwareMap.dcMotor.get("right_motor");

        DcMotor[] motors = {left_motor, right_motor};
        for (DcMotor motor : motors) {
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        Servo arm = hardwareMap.servo.get("arm");



        // wait for start button
        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            if (y < 0) {
                y /= 2; // slowing it down when reversing
            }
            double x = gamepad1.right_stick_x;

            double left_power = (y - x / 2);
            double right_power = (y + x / 2);
            double max_power =  Math.max(Math.abs(left_power), Math.abs(right_power));

            // Scaling so that nothing gets accidentally clipped
            if (max_power > 1) {
                left_power /= max_power;
                right_power /= max_power;
            }

            left_motor.setPower(left_power);
            right_motor.setPower(right_power);

            if (gamepad1.right_bumper) {
                arm.setPosition(0.0);
            } else {
                arm.setPosition(0.5);
            }

            telemetry.addData("left motor", left_motor.getPower());
            telemetry.addData("right motor", right_motor.getPower());
            telemetry.addData("arm position", arm.getPosition());
            telemetry.update();

        }
    }
}


