/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Encoder")
public class EncoderDrive extends LinearOpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frMotor.setTargetPosition(Constants.TICKS_PER_METER * 10);
        flMotor.setTargetPosition(Constants.TICKS_PER_METER * 10);
        brMotor.setTargetPosition(Constants.TICKS_PER_METER * 10);
        blMotor.setTargetPosition(Constants.TICKS_PER_METER * 10);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        setAllPower(0.25);

        while(opModeIsActive() && (frMotor.isBusy() || flMotor.isBusy() || brMotor.isBusy() || blMotor.isBusy())) {
            idle();
        }

        setAllPower(0);
    }


    private void initialize() {
        frMotor = hardwareMap.dcMotor.get("frMotor");
        flMotor = hardwareMap.dcMotor.get("flMotor");
        brMotor = hardwareMap.dcMotor.get("brMotor");
        blMotor = hardwareMap.dcMotor.get("blMotor");

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setMotorMode(DcMotor.RunMode runMode) {
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);
    }

    private void setAllPower(double power) {
        frMotor.setPower(power);
        flMotor.setPower(power);
        brMotor.setPower(power);
        blMotor.setPower(power);
    }
}
*/