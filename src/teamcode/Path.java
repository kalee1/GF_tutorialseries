// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Path Following
package teamcode;

import processing.core.PVector;

import java.util.ArrayList;


public class Path
{

    // A Path is an arraylist of points (PVector objects)
    ArrayList<PVector> points;
    ArrayList<Float> headings;
    ArrayList<Float> maxspeeds;

    // A path has a pathDeadband, i.e how far is it ok for the boid to wander off
    double pathDeadband;
    double pointRadius = 10;

    public Path()
    {
        // Arbitrary pathDeadband of 20
        pathDeadband = 2;
        points = new ArrayList<PVector>();
        points.clear();
        headings = new ArrayList<Float>();
        headings.clear();
        maxspeeds = new ArrayList<Float>();
        maxspeeds.clear();
    }

    // Add a point to the path
    public void addPoint(float x, float y)
    {
        PVector point = new PVector(x, y);
        points.add(point);
        headings.add((float)0);
        maxspeeds.add((float)4.25);
    }

    public void addPoint( float x, float y, float heading)
    {
        PVector point = new PVector(x, y);
        points.add(point);
        headings.add(heading);
        maxspeeds.add((float)4.25);
    }

    public void addPoint( float x, float y, float heading, float maxspeed)
    {
        PVector point = new PVector(x, y);
        points.add(point);
        headings.add(heading);
        maxspeeds.add(maxspeed);
    }

    public PVector getStart()
    {
        return points.get(0);
    }
    public float getInitialHeading() { return headings.get(0);}

    public PVector getEnd()
    {
        return points.get(points.size() - 1);
    }


}
