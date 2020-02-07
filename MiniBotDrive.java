package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// TeleOp for minibot.

@TeleOp
public class MiniBotDrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor left_motor = hardwareMap.dcMotor.get("left_motor");
        DcMotor right_motor = hardwareMap.dcMotor.get("right_motor");

        // Creating an array of motors so we can iterate over it.
        DcMotor[] motors = {left_motor, right_motor};
        // Initializing the motors.
        for (DcMotor motor : motors) {
            // REV HD Hex encoder counts 2240 per rotation.
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        DistanceSensor left_distance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor right_distance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor[] sensors = {left_distance, right_distance};


        // wait for start button
        waitForStart();

        while (opModeIsActive()) {

            for (DcMotor motor : motors) {
                telemetry.addData(motor.getDeviceName(), motor.getPower());
            }
            for (DistanceSensor sensor : sensors) {
                telemetry.addData(sensor.getDeviceName(), sensor.getDistance(DistanceUnit.CM));
            }
            telemetry.update();

        }
    }
}


