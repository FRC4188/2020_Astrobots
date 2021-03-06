
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

//2.286
@Autonomous(name = "ParkingRedLeft")
public class ParkingRedLeft extends LinearOpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor, verticalArm;
    private DcMotorEx shooterMotor;
    private CRServo horizontalArm;
    private DcMotorEx shooterMotorEx;


    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //
        waitForStart();

        horizontalArm.setPower(-1.0);

        drivetrain(1.4, 0,0);
        setAllPower(1.0);

        while (opModeIsActive() && isRobotBusy()) {
            //checkMotors();
            idle();
        }
        setAllPower(0.0);

        resetStartTime();

        //-------------------------------------------------------------

        drivetrain(0, 0,-0.042);
        setAllPower(1.0);

        while (opModeIsActive() && isRobotBusy()) {
            //checkMotors();
            idle();
        }
        setAllPower(0.0);

        resetStartTime();

        //-------------------------------------------------------------



        runtime.reset();

        while (opModeIsActive()){
            double time = runtime.seconds();
            if (time > 0 && time < 13){
                shooterMotorEx.setVelocity(1350);
                telemetry.addData("shootervel", shooterMotorEx.getVelocity());
                telemetry.update();

            }
            if (time > 13 && time < 13.75){
                magazineMotor.setPower(1.0);
                intakeMotor.setPower(-1.0);
            }
            if (time > 13.75 && time < 14.5){
                magazineMotor.setPower(0.0);
            }
            if (time > 14.5 && time < 15.25){
                magazineMotor.setPower(1.0);
            }
            if (time > 15.25 && time < 16){
                magazineMotor.setPower(0.0);
            }
            if (time > 16 && time < 17){
                magazineMotor.setPower(1.0);
            }
            if (time > 17 && time < 18){
                intakeMotor.setPower(0.0);
                magazineMotor.setPower(0.0);
                shooterMotor.setVelocity(0);
            }
            if (time > 18 && time < 19){
                drivetrain(0.25, 0,0);
                setAllPower(1.0);

                while (opModeIsActive() && isRobotBusy()) {
                    //checkMotors();
                    idle();
                }
                setAllPower(0.0);
            }


        }



    }

    private void initialize() {

        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");


        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalArm = hardwareMap.get(CRServo.class, "hArm");
        verticalArm = hardwareMap.get(DcMotor.class, "vArm");
        verticalArm.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotorEx = (DcMotorEx) shooterMotor;


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
    private void checkMotors(){
        if(!frMotor.isBusy()) {
            frMotor.setPower(0);
        }

        if(!flMotor.isBusy()) {
            flMotor.setPower(0);
        }

        if(!brMotor.isBusy()) {
            brMotor.setPower(0);
        }

        if(!blMotor.isBusy()) {
            blMotor.setPower(0);
        }
    }

    private boolean isRobotBusy() {
        return blMotor.isBusy();
    }

    /**
     * Increment a value by delta and return the new value.
     *
     * @param  forwardDistance   the distance, in meters, the robot should move forward
     * @param  strafeDistance    the distance, in meters, the robot should strafe sideways
     * @param  rotationDistance  the angle to rotate the robot
     */
    private void drivetrain(double forwardDistance, double strafeDistance, double rotationDistance) {

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frMotor.setTargetPosition(frMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance - strafeDistance - rotationDistance)));
        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance + strafeDistance + rotationDistance)));
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance + strafeDistance - rotationDistance)));
        blMotor.setTargetPosition(blMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance - strafeDistance + rotationDistance)));

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void driveSeconds(double forward, double strafe, double rotation) {
        frMotor.setPower(forward - strafe - rotation);
        flMotor.setPower(forward + strafe + rotation);
        brMotor.setPower(forward + strafe - rotation);
        blMotor.setPower(forward - strafe + rotation);
    }

}
