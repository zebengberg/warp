package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// WARP Dec 2019

@Autonomous
public class AutoWARPBlueBlocks extends LinearOpMode {
    DcMotor front_left_wheel;
    DcMotor back_left_wheel;
    DcMotor back_right_wheel;
    DcMotor front_right_wheel;

    DcMotor[] motors;

    Servo left_arm;
    Servo right_arm;

    ColorSensor left_color;
    ColorSensor right_color;
    DistanceSensor distance;





    @Override
    public void runOpMode() {
        // DC motors for holonomic drive.
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");

        // Creating an array of motors so we can iterate over it.
        motors = new DcMotor[] {front_left_wheel, back_left_wheel, back_right_wheel, front_right_wheel};




        for (DcMotor motor : motors) {
            // REV HD Hex encoder counts 2240 per rotation.
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Servos for little arms
        left_arm = hardwareMap.servo.get("left_arm");
        right_arm = hardwareMap.servo.get("right_arm");
        left_arm.resetDeviceConfigurationForOpMode();
        right_arm.resetDeviceConfigurationForOpMode();
        left_arm.setDirection(Servo.Direction.FORWARD);
        right_arm.setDirection(Servo.Direction.REVERSE);

        // Sensors
        left_color = hardwareMap.get(ColorSensor.class, "left_color");
        right_color = hardwareMap.get(ColorSensor.class, "right_color");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        // wait for start button
        waitForStart();
        for (DcMotor motor : motors) {
            motor.setTargetPosition(1000);
            motor.setPower(1);
        }

        if (opModeIsActive()) {

            goForward(10);
            sleep(500);
            goForward(20);
            sleep(500);
            goForward(10);

            float reds[] = {left_color.red(), right_color.red()};
            float greens[] = {left_color.green(), right_color.green()};
            float blues[] = {left_color.blue(), right_color.blue()};
            float alphas[] = {left_color.alpha(), right_color.alpha()};

            telemetry.addData("Distance (cm)", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("Red  ", reds);
            telemetry.addData("Green", greens);
            telemetry.addData("Blue ", blues);
            telemetry.addData("Alpha", alphas);
            telemetry.update();

        }
    }

    public void goForward(double cm) {
        int position = (int) (cm * Math.sqrt(0.5) * 2240.0 / 28.0);

        front_left_wheel.setTargetPosition(position);
        front_left_wheel.setPower(1);
        back_left_wheel.setTargetPosition(position);
        back_left_wheel.setPower(1);

        front_right_wheel.setTargetPosition(-position);
        front_right_wheel.setPower(-1);
        back_right_wheel.setTargetPosition(-position);
        back_right_wheel.setPower(-1);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while ((front_left_wheel.isBusy() || front_right_wheel.isBusy() || back_right_wheel.isBusy()
                || back_left_wheel.isBusy()) && opModeIsActive()) {
            telemetry.addData("front left", front_left_wheel.getCurrentPosition());
            telemetry.addData("front right", front_right_wheel.getCurrentPosition());
            telemetry.addData("back left", back_left_wheel.getCurrentPosition());
            telemetry.addData("back right", back_right_wheel.getCurrentPosition());
            telemetry.update();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }
}

