package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Parking1")
public class Parking1 extends LinearOpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor, shooterMotor;
    private CRServo arm;

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        runtime.reset();
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //-Constants.TICKS_PER_METER * 1.778)


        frMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 1.72));
        flMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 1.72));
        brMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 1.72));
        blMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 1.72));

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        runtime.reset();
        arm.setPower(0.2);
        wait(2000);
        arm.setPower(0.0);

        setAllPower(1.0);

        while(opModeIsActive() && (frMotor.isBusy() || flMotor.isBusy() || brMotor.isBusy() || blMotor.isBusy())) {
            idle();
        }

        setAllPower(0);

        shooterMotor.setPower(0.8);
        wait(6000);
        magazineMotor.setPower(1.0);
        wait(8000);



    }

    private void initialize() {

        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        arm = hardwareMap.get(CRServo.class, "wobbleArm");

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
}

    private void setMotorMode(DcMotor.RunMode runMode) {
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);
    }

    private void setAllPower(double power) {
        frMotor.setPower(power);
        flMotor.setPower(power - 0.06);
        brMotor.setPower(power);
        blMotor.setPower(power - 0.06);
    }

}
/*
        resetStartTime();

        while (opModeIsActive() && getRuntime() < 3) {
            idle();
        }
        //
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER * 0.205));
        flMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER * 0.205));
        brMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER * 0.205));
        blMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER * 0.205));

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        setAllPower(0.5);
        while (opModeIsActive() && (frMotor.isBusy())) {
            idle();
        }
        setAllPower(0);
*/