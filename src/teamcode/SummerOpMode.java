package teamcode;


import processing.core.PVector;

public class SummerOpMode extends OpMode{

    Vehicle theVehicle = null;
    Path myPath;

    public SummerOpMode(Vehicle aVehicle)
    {
        theVehicle = aVehicle;
    }

    @Override
    public void init()
    {
        newPath();
    }

    @Override
    public void loop()
    {
        theVehicle.follow(myPath);

        theVehicle.update();

    }



    public void newPath()
    {
        // A path is a series of connected points
        // A more sophisticated path might be a curve
        myPath = new Path();
        myPath.addPoint(140,260, -90, (float)3.5);
        myPath.addPoint(30,150, -90, 4);
        myPath.addPoint(30,150, -90, 4);
        myPath.addPoint(30,50,-90, 4);
        myPath.addPoint(30, 148,-225, 4);
        myPath.addPoint(140,265,-225, 2);
        myPath.addPoint( 125, 300,-225, 2);
        myPath.addPoint(140, 275,-255, 2);
        myPath.addPoint( 130, 225,-245, 2);
        myPath.addPoint(140, 275,-225, 2);
        myPath.addPoint( 130, 295,-225, 2);
        myPath.addPoint(140, 275,-255, 2);
        myPath.addPoint( 130, 225,-245, 2);
        myPath.addPoint(140, 275,-225, 2);
        myPath.addPoint( 130, 295,-225, 2);

    }
}
