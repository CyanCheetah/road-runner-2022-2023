package org.firstinspires.ftc.teamcode.drive;//Hollow World
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.logging.Level;
import com.qualcomm.robotcore.hardware.configuration.UnspecifiedMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;


//------------------------------------------------------------------------------//
@TeleOp(name="CyanCheetahOp", group="Linear Opmode")

public class CyanCheetahOp extends LinearOpMode {


    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;

    }


    private static DcMotor frontl = null;
    private static DcMotor frontr = null;
    private static DcMotor bottoml = null;
    private static DcMotor bottomr = null;
    private static DcMotor ElevatorMotor = null;

    private static CRServo servo = null;
    private static CRServo servo1 = null;

    // private static DcMotor shooter = null;
    //   private static CRServo frontservo = null;
    // private DistanceSensor distanceSensor;
//---\
//   private static CRServo lowestServo = null;
    //  private static CRServo middleServo = null;
    // private static CRServo highestServo = null;
    // private static DcMotor ArmMotor = null;
    // private static CRServo ServoArm = null;
    // private static ColorSensor ColorSensorRPM = null;
//-------------------------------------------------------------------------------//
    enum PowerLevel {MAX, HALF, QUARTER, STOP}

    ;

    // Declare OpMode members/constants.
    private ElapsedTime runtime = new ElapsedTime();
    //private static final double ADJUST_DIAGONAL = 0.40; ***Not used anymore***
    private static double MOTOR_ADJUST = 0.75;
    //Defining Motor Arm Movement
    private static double SMALL_GEAR_TEETH = 40; //Smaller gear is directly on the motor
    private static double LARGE_GEAR_TEETH = 120; //Larger gear is connected to the arm
    private static double GEAR_RATIO = (LARGE_GEAR_TEETH / SMALL_GEAR_TEETH); //States the gear ratio based on the teeth of gears used
    private static double ORIBITAL_60_PULSE_PER_ROTATION = 1680; //Motor is an AndyMark Neverest 60 GearMotor
    private static double ARM_PULSE_PER_ROTATION = ORIBITAL_60_PULSE_PER_ROTATION * GEAR_RATIO; //Amount of rotation per motor to gears
    private static double DEGREES_PER_TICK = 360 / ARM_PULSE_PER_ROTATION; //Converts Radians into Degrees
    //Power for Motor Arm
    private static double MOTOR_ARM_MAX_POWER = .7;
    private static double MOTOR_ARM_MIN_POWER = .3;
    private static double ARM_HOLDING_POWER = .15;
    private static int OPT_PICKUP_HEIGHT = 186;
    private static int OPT_TRAV_HEIGHT = 459;
    private final long BILLION = 1000000000;
    private static double SIDEWAYS_DRIFT_CORRECTION = 1.125;
    private int x = 1;
    private int I = 0;
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean ypressed = false;
    //double initialServoPosition = ServoArm.getPosition();


    @Override
    public void runOpMode() {

        PowerLevel powerLevel = PowerLevel.HALF.QUARTER;
        //double initialServoPosition = ServoArm.getPosition();//Starts the robot wheels at MAX power level
        frontl = hardwareMap.get(DcMotor.class, "frontl");
        frontr = hardwareMap.get(DcMotor.class, "frontr");
        bottoml = hardwareMap.get(DcMotor.class, "bottoml");
        bottomr = hardwareMap.get(DcMotor.class, "bottomr");
        servo = hardwareMap.get(CRServo.class, "servo");

        servo1 = hardwareMap.get(CRServo.class, "servo1");
        ElevatorMotor = hardwareMap.get(DcMotor.class, "ElevatorMotor");
        //    shooter= hardwareMap.get(DcMotor.class, "shooter");

        frontl.setDirection(DcMotor.Direction.FORWARD);
        frontr.setDirection(DcMotor.Direction.REVERSE);
        bottoml.setDirection(DcMotor.Direction.FORWARD);
        bottomr.setDirection(DcMotor.Direction.REVERSE);
        ElevatorMotor.setDirection(DcMotor.Direction.FORWARD);


        ElevatorMotor.setTargetPosition(0);


        ElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Brake immedietly after joystick hits 0 instead of coasting down
        frontl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottoml.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //---


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//------------------------------------------------------------------ Start of Match ---------------------------------------------------
        runtime.reset();
        frontl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottoml.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottoml.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Setup a variable for each drive wheel
        double frontlPower = 0;
        double frontrPower = 0;
        double bottomlPower = 0;
        double bottomrPower = 0;
        double ArmMotorPower = 0;

        double ElevatorMotorPower = 1;
        double triggerPowerAdjust = 1;


//anyd is baldsm
        while (opModeIsActive()) {  //While Teleop is in session


//          ************************************************ GAMEPAD 2 CONTROLS ************************************************


            //3 touch// 440 =  optimum pickup// 200 = bottom// 400 = optimum travel height/ 1880 drop height// 2958 drop height l



            if (gamepad2.dpad_up) {
                ElevatorMotor.setTargetPosition(ElevatorMotor.getCurrentPosition()+90);

                ElevatorMotor.setPower(.7);

            }
            else if (gamepad2.dpad_down) {
                ElevatorMotor.setTargetPosition(ElevatorMotor.getCurrentPosition()-100);

                ElevatorMotor.setPower(-.5);
            }





            if (gamepad2.right_trigger>.1) {
                servo.setPower(-1);
                servo1.setPower(1);

            } else if (gamepad2.left_trigger>.1) {
                servo.setPower(1);
                servo1.setPower(-1);

            } else {
                servo.setPower(0);
                servo1.setPower(0);

            }

            if (gamepad2.x) {
                //ElevatorMotor.getCurrentPosition();
                ElevatorMotor.setTargetPosition(3200);
                ElevatorMotor.setPower(.7);

            } else if (gamepad2.y) {
                ElevatorMotor.setTargetPosition(0);
                ElevatorMotor.setPower(-.5);

            }else if (gamepad2.a) {
                ElevatorMotor.setTargetPosition(2150);
                ElevatorMotor.setPower(.8);

            }
            else if (gamepad2.b) {
                ElevatorMotor.setTargetPosition(1350);
                ElevatorMotor.setPower(-.6);

            }

/*
                ElevatorMotor.setTargetPosition(5);
                ElevatorMotor.setPower(.6);

 */

/*
            if (gamepad1.right_trigger > 0) {
                triggerPowerAdjust = .4;
            } else {
                triggerPowerAdjust = 1;
            }

 */


            //See the Desmos for an explanation, this is code that's basically modified from what they (FTC) gave us
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x * 0.5;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            v1 = v1 * MOTOR_ADJUST * triggerPowerAdjust;
            v2 = v2 * MOTOR_ADJUST * triggerPowerAdjust;
            v3 = v3 * MOTOR_ADJUST * triggerPowerAdjust;
            v4 = v4 * MOTOR_ADJUST * triggerPowerAdjust;
            frontl.setPower(v1 * .95);
            frontr.setPower(v2 * .95);
            bottoml.setPower(v3 * .95);
            bottomr.setPower(v4 * .95);
            //  telemetry.addData("Motor Power", "v1 (%.2f), v2 (%.2f) v3 (%.2f) v4 (%.2f)", v1,v2,v3,v4);
            //  telemetry.addData("R","%.2f",r);


//          ************************************************ GAMEPAD 2 CONTROLS ************************************************


            //Telemetry
            telemetry.addData("Elevator Motor Encoder Ticks", ElevatorMotor.getCurrentPosition());
            telemetry.addData("FrontL", -1 * frontl.getCurrentPosition());
            telemetry.addData("FrontR", -1 * frontr.getCurrentPosition());

            telemetry.addData("BackL", -1 * bottoml.getCurrentPosition());
            telemetry.addData("BackR", -1 * bottomr.getCurrentPosition());


            telemetry.addData("Battery Voltage", getBatteryVoltage());
            telemetry.update();

        }//While opMode
    }//void runOpMode
}







/*
Final Notes from Andy: Apressed sets the slide puller to optimum pickup, Bpressed sets slide puller to optimum travel and same concept with elevator motors just with Xpressed and Ypressed. Just need to do cleanup and redo elevator motors since we added another one
*/