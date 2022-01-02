package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;


@Disabled
@TeleOp(name="TeleOpFinal Fake", group="Iterative Opmode")
public class TeleOpFinal extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftForward, rightForward, leftBack, rightBack, carouselMotor, intakeMotor, armMotor;
    private double carouselSpeed = 0.70;
    private double intakeSpeed = 1.00;
    private Servo turnServo, clawServo;
    private double speed = 1.00;



     // Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /**
        * Initialize the hardware variables. Note that the strings used here as parameters
        * to 'get' must correspond to the names assigned during the robot configuration
        * step (using the FTC Robot Controller app on the phone).
        */
         leftForward  = hardwareMap.get(DcMotor.class, "LF Motor");
        rightForward = hardwareMap.get(DcMotor.class, "RF Motor");
        leftBack = hardwareMap.get(DcMotor.class, "LB Motor");
        rightBack = hardwareMap.get(DcMotor.class, "RB Motor");
        carouselMotor = hardwareMap.get(DcMotor.class, "Carousel Motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        armMotor = hardwareMap.get(DcMotor.class, "Arm Motor");

        clawServo = hardwareMap.get(Servo.class, "Claw Servo");
        turnServo = hardwareMap.get(Servo.class, "Turn Servo");

        /**
        * Most robots need the motor on one side to be reversed to drive forward
        * Reverse the motor that runs backwards when connected directly to the battery
        */

        leftForward.setDirection(DcMotor.Direction.FORWARD);
        rightForward.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        carouselMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        clawServo.setDirection(Servo.Direction.FORWARD);
        turnServo.setDirection(Servo.Direction.FORWARD);

        /*
        leftForward.setDirection(DcMotor.Direction.REVERSE);
        rightForward.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        carouselMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        clawServo.setDirection(Servo.Direction.FORWARD);
        turnServo.setDirection(Servo.Direction.FORWARD);
   */
        // Tell the driver that  is complete.
        telemetry.addData("Status", "Initialized");
    }

     // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

     // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }


      //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double LFPower, LBPower, RFPower, RBPower, xValue, turnValue, yValue;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        yValue = gamepad1.left_stick_y;
        turnValue = gamepad1.right_stick_x;
        xValue = gamepad1.left_stick_x;

        LFPower = Range.clip(-yValue + turnValue + xValue,-1,1);
        LBPower = Range.clip(-yValue + turnValue - xValue,-1,1);
        RBPower = Range.clip(-yValue - turnValue + xValue,-1,1);
        RFPower = Range.clip(-yValue - turnValue - xValue,-1,1);

        //applying the ramping up and ramping down features
        if (LFPower < 0){
            LFPower = (float) -Math.pow(Math.abs(LFPower),2);
        } else if (LFPower > 0){
            LFPower = (float) Math.pow(Math.abs(LFPower),2);
        }

        if (LBPower < 0){
            LBPower = (float) -Math.pow(Math.abs(LBPower),2);
        } else if (LBPower > 0){
            LBPower = (float) Math.pow(Math.abs(LBPower),2);
        }

        if (RFPower < 0){
            RFPower = (float) -Math.pow(Math.abs(RFPower),2);
        } else if (RFPower > 0){
            RFPower = (float) Math.pow(Math.abs(RFPower),2);
        }

        if (RBPower < 0){
            RBPower = (float) -Math.pow(Math.abs(RBPower),2);
        } else if (RBPower > 0){
            RBPower = (float) Math.pow(Math.abs(RBPower),2);
        }


        if (gamepad1.a){
            speed = 0.3;
        } else{
            speed = 1.00;
        }

        //setting the powers for each of the motors
        leftForward.setPower(Range.clip(LFPower, -speed, speed));
        leftBack.setPower(Range.clip(LBPower, -speed, speed));
        rightForward.setPower(Range.clip(RFPower, -speed, speed));
        rightBack.setPower(Range.clip(RBPower, -speed, speed));


        if(gamepad1.right_bumper) {
            carouselMotor.setPower(carouselSpeed);
        }
        else if(gamepad1.left_bumper) {
            carouselMotor.setPower(-carouselSpeed);
        }
        else {
            carouselMotor.setPower(0);
        }

        if(gamepad1.right_trigger > 0.0) {
            intakeMotor.setPower(intakeSpeed);
        }
        else if(gamepad1.left_trigger > 0.0){
            intakeMotor.setPower(-intakeSpeed);
        }
        else {
            intakeMotor.setPower(0);
        }

        if(gamepad2.left_stick_y > 0.0) {
            armMotor.setPower(0.4);

        }
        else if(gamepad2.left_stick_y < 0.0) {
            armMotor.setPower(-0.4);

        }
        else {
            armMotor.setPower(0.0);
        }
/*
        if(gamepad1.dpad_up) {
            armMotor.setPower(0.6);

        }
        else if(gamepad1.dpad_down) {
            armMotor.setPower(-0.6);

        }
        else {
            armMotor.setPower(0.0);
        }
*/
        if(gamepad2.a) {
            //turnServo.setPosition(Servo.MAX_POSITION);
            turnServo.setPosition(0.65);
        }
        if(gamepad2.b) {
            turnServo.setPosition(0.175);
        }
/*
        if(gamepad1.x) {
            clawServo.setPosition(Servo.MIN_POSITION);
            turnServo.setPosition(0.5);
        }
        if(gamepad1.y) {
            turnServo.setPosition(Servo.MAX_POSITION);
            clawServo.setPosition(0.3);
        }
        */

        if(gamepad2.x) {
            clawServo.setPosition(Servo.MIN_POSITION);
            turnServo.setPosition(0.175);
        }
        if(gamepad2.y) {
            turnServo.setPosition(0.65);
            clawServo.setPosition(0.3);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Carousel Speed", "I hate all of u: " + intakeSpeed);
    }


     // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
