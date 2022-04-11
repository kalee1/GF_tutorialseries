package Main;

import processing.core.PVector;
import tbdServer.ComputerDebugging;
import tbdServer.FloatPoint;
import teamcode.SummerOpMode;
import teamcode.OpMode;
import teamcode.Vehicle;

public class Main
{
    public static void main(String[] args)
    {
        new Main().run();
    }

    PVector initialPos = new PVector(158, 200);
    Vehicle theVehicle = new Vehicle(initialPos, -45, (float)4.25, (float)1);
//    Path myPath;
    float height = 358;
    float width = 358;

    /**
     * The program runs here
     */
    public void run()
    {
        ComputerDebugging computerDebugging = new ComputerDebugging();
//        Robot theRobot = new Robot();

        OpMode opMode = new SummerOpMode( theVehicle );
        opMode.init();

        ComputerDebugging.clearLogPoints();

        long startTime = System.currentTimeMillis();
        try
        {
            Thread.sleep(1000);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
        while (true)
        {
            opMode.loop();
//            theVehicle.follow(myPath);
            try
            {
                Thread.sleep(50);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            //theRobot.update();
//            theVehicle.update();
            ComputerDebugging.sendRobotLocation(theVehicle);
            ComputerDebugging.sendLogPoint(new FloatPoint(theVehicle.getX(), theVehicle.getY()));
            ComputerDebugging.markEndOfUpdate();
        }
    }
}
