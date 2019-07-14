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
    // A path has a pathDeadband, i.e how far is it ok for the boid to wander off
    double pathDeadband;
    double pointRadius = 15;

    public Path()
    {
        // Arbitrary pathDeadband of 20
        pathDeadband = 2;
        points = new ArrayList<PVector>();
        points.clear();
        headings = new ArrayList<Float>();
        headings.clear();
    }

    // Add a point to the path
    public void addPoint(float x, float y)
    {
        PVector point = new PVector(x, y);
        points.add(point);
        headings.add((float)0);
    }

    public void addPoint( float x, float y, float heading)
    {
        PVector point = new PVector(x, y);
        points.add(point);
        headings.add(heading);
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
