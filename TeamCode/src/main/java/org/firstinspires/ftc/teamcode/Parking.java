
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Parking", group="Linear Opmode")
public class Parking extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frMotor, flMotor, brMotor, blMotor;
    int ticksPerMeter = (int) Math.round(6111.5498);

    double WHEEL_DIAM = 0.075;
    double WHEEL_CIRC = Math.PI * WHEEL_DIAM;
    double MOTOR_CPR = 1440;
    int TICKS_PER_METER = (int) Math.round(MOTOR_CPR / WHEEL_CIRC);

    @Override
    public void runOpMode () throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frMotor.setTargetPosition(ticksPerMeter);
        brMotor.setTargetPosition(ticksPerMeter);
        flMotor.setTargetPosition(ticksPerMeter);
        blMotor.setTargetPosition(ticksPerMeter);

        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Hello", "Hello");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        frMotor.setPower(0.25);
        brMotor.setPower(0.25);
        flMotor.setPower(0.25);
        blMotor.setPower(0.25);

        while(opModeIsActive() && (frMotor.isBusy() || brMotor.isBusy() || blMotor.isBusy() || brMotor.isBusy())) {
            idle();
        }

        frMotor.setPower(0);
        brMotor.setPower(0);
        flMotor.setPower(0);
        blMotor.setPower(0);

        resetStartTime();

    }
}