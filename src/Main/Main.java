package Main;

import processing.core.PVector;
import sbfServer.ComputerDebugging;
import sbfServer.FloatPoint;
import sbfServer.Robot;
import teamcode.MyOpMode;
import teamcode.OpMode;
import teamcode.Path;
import teamcode.Vehicle;

import java.util.Random;

public class Main
{
    public static void main(String[] args)
    {
        new Main().run();
    }

    PVector initialPos = new PVector(158, 200);
    Vehicle car = new Vehicle(initialPos, -45, (float)4.25, (float).75);
    Path myPath;
    float height = 358;
    float width = 358;

    /**
     * The program runs here
     */
    public void run()
    {
        //this is a test of the coding
        ComputerDebugging computerDebugging = new ComputerDebugging();
        Robot robot = new Robot();


        OpMode opMode = new MyOpMode();
        opMode.init();
        newPath();

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

            //opMode.loop();
            car.follow(myPath);

            try
            {
                Thread.sleep(50);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            //robot.update();
            car.update();
            ComputerDebugging.sendRobotLocation(car);
            ComputerDebugging.sendLogPoint(new FloatPoint(car.getX(), car.getY()));
            ComputerDebugging.markEndOfUpdate();
        }
    }

    public void newPath()
    {
        // A path is a series of connected points
        // A more sophisticated path might be a curve
        Random randomFloat = new Random();
        myPath = new Path();
        myPath.addPoint(140,260, -90, 3);
        myPath.addPoint(27,150, -90, 9);
        myPath.addPoint(27,50,-90, 12);
        myPath.addPoint(27, 148,-225, 12);
        myPath.addPoint(140,265,-225, 2);
        myPath.addPoint( 125, 300,-225, 2);
        myPath.addPoint(150, 275,-225, 2);
        myPath.addPoint( 130, 225,-225, 2);
        myPath.addPoint(150, 275,-225, 2);
        myPath.addPoint( 130, 295,-225, 2);
        myPath.addPoint(150, 275,-225, 2);
        myPath.addPoint( 130, 225,-225, 2);
        myPath.addPoint(150, 275,-225, 2);
        myPath.addPoint( 130, 295,-225, 2);

    }
}
