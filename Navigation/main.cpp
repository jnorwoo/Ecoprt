#include <iostream>
#include <complex>      // std::complex, std::polar
#include <deque>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>       /* pow */

using namespace std;
#define False 0;
#define True 1;

std::vector<std::string> &split( std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split( std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

// Class to represent points.
class Point {
private:
        double xval, yval;
public:
        // Constructor uses default arguments to allow calling with zero, one,
        // or two values.
        Point(double x = 0.0, double y = 0.0) {
                xval = x;
                yval = y;
        }

        // Extractors.
        double x() { return xval; }
        double y() { return yval; }

        // Distance to another point.  Pythagorean thm.
        double dist(Point other) {
                double xd = xval - other.xval;
                double yd = yval - other.yval;
                return sqrt(xd*xd + yd*yd);
        }

        // Add or subtract two points.
        Point add(Point b)
        {
                return Point(xval + b.xval, yval + b.yval);
        }
        Point sub(Point b)
        {
                return Point(xval - b.xval, yval - b.yval);
        }

        // Move the existing point.
        void move(double a, double b)
        {
                xval += a;
                yval += b;
        }

        // Print the point on the stream.  The class ostream is a base class
        // for output streams of various types.
        void print(ostream &strm)
        {
                strm << "(" << xval << "," << yval << ")";
        }
};

int updateRate = 83;
int TILE_SIZE = 256;
int prevx = 0;
int prevy = 0;
int pointPerFoot = 1;
int topAcceptableFeet = 10 * pointPerFoot;
int bottomAcceptableFeet = 10 * pointPerFoot;

int newroverLatlng = 0;
int dx = 1 * pointPerFoot;
int dy = 0;
int roverSpeed = 3 * pointPerFoot;
complex<double> heading = polar(0.0, 0.0);
int turningAngle = .07854 * 2;  // turn 90 degrees in 5 seconds if updating at intervals of 2hz
Point accetableRangeTopWayPointWorldCoordinate = Point(0, 0);
Point accetableRangeBottomWayPointWorldCoordinate = Point(0, 0);

Point accetableRangeTopstartingPointWorldCoordinate = Point(0, 0);
Point accetableRangeBottomstartingPointWorldCoordinate = Point(0, 0);
Point roverWorldCoordinate = 0;
Point WayPointWorldCoordinate = 0;
Point startingPointWorldCoordinate = 0;
Point rover = 0;
int waypointAcceptableRange = 3;
int lineNumber = 1;
deque<int> waypointQueue ;
deque<int> headingQueue ;
deque<Point> navigationQueue;

int wasAtWaypoint = False;  // should not start at waypoint
int roverNewHeadingDebounce = False;  // not debouncing
int roverNewHeadingDebounceTime = 3000;  // find a new heading in 3 seconds
deque<Point> roverPostions ;

Point  startingPoint = 0;
std::string::size_type sz;     // alias of size_t
double gpsX;
double gpsY;



void iniatilze(){
    WayPointWorldCoordinate = Point(200, 200); // set a waypoint at 200 meter 200 meteres
    navigationQueue.push_back(WayPointWorldCoordinate); // add the waypoint to the collection of waypoints
}


double distanceFunc(Point point1,Point point2){
   // # distance formula returns in point
    return sqrt(pow((point2.x() - point1.x()), 2) + pow((point2.y() - point1.y()), 2));
}
int atWayPoint(){
//    # if distance betwen the way point and the rover is less than 3 feet
    if (distanceFunc(roverWorldCoordinate, WayPointWorldCoordinate) < waypointAcceptableRange * pointPerFoot){
        if (navigationQueue.size() > 1){
            startingPoint = navigationQueue.front();
            navigationQueue.pop_front();

}
        else{
            startingPoint = navigationQueue[0];
            heading = polar(0,0);
        }
//        if (len(navigationQueue) >= 1):
//            startingPointWorldCoordinate = startingPoint
//            WayPointWorldCoordinate = navigationQueue[0]
//        else:
//            heading.r = 0  # stop car

        return True
}
    else{
        return False
}

    //    # if the previous rover position distance to the waypoint is smaller
      //  # than the current than its go away from the waypoint
}

double slope(Point a, Point b){
//    # y1 - y2 / x2 - x1
//    # console.log((a.y - b.y)/(a.x - b.x))
    try{
        return (a.y() - b.y()) / (a.x() - b.x());
    }
    catch(...){
//        return double('inf')  # math.inf #"Error" a.y  / a.x
        }
    return 0.0;
}


double normalizeAngle(double angle){ // angle between 0 and 2pi
    return fmod((angle + (2 * M_PI)) , (2 * M_PI)); // fmod for modulus with doubles
}

double flipHorizontally(double angle){
    double n = floor(angle / (M_PI));
    return normalizeAngle((M_PI) - fmod(angle , (M_PI)) + n * (M_PI));
}

double rotateLeft(double angle){ // rotate angle 90 degrees counter clockwise
    return normalizeAngle(angle + (M_PI / 2));
}
double bearingToUnits(double bearing){
    return flipHorizontally(rotateLeft(bearing));
}
int rotateToParrallel(){
    double m1 = slope(roverPostions[0], WayPointWorldCoordinate);// slope of the wapoint and the current postion
    double m2 = slope(roverPostions[0], roverPostions[1]); // slope of the rovers heading.
    double bearing;
    try{
        if ( !isinf((float)m2) && !isinf((float)m1)){ // make sure that the slopes are not infinite
            if (roverPostions[0].x() < roverPostions[1].x()){
                bearing = atan2(m2, 1);
                }
            else{
                bearing = atan2(m2, 1) + M_PI;
                }
              heading = polar(fabs(heading),normalizeAngle(bearingToUnits(bearing)));
//            #print str(heading.theta)
//            tanTheta = (m2 - m1) / (1 + m1 * m2)
//            if (not math.isinf(tanTheta)):
//                theta = math.atan2(tanTheta, 1)
//                if (not math.isinf(theta)):
//                    if (theta):
//                        return theta
//                    else:
//                        return 0
//                else:
//                    return 0
            }
//        else:
//            return 0
        }
    catch(...){
//        #print "Unexpected error:", sys.exc_info()[0]
        return 0;  // print "error"
    }
    return 0;
    }

void Turn(){
        try{
            turningAngle = 1 * abs(rotateToParrallel());
        }
        catch(...)
        {
        }
//        if (startingPointWorldCoordinate.x < WayPointWorldCoordinate.x):
//            if (aboveLine(startingPointWorldCoordinate, WayPointWorldCoordinate)):
//                heading.theta += turningAngle
//                print  "d"
//
//
//            else:
//                heading.theta -= turningAngle
//                print  "a"
//        else:
//            if (aboveLine(startingPointWorldCoordinate, WayPointWorldCoordinate)):
//                heading.theta -= turningAngle
//                print  "a"
//
//            else:
//                heading.theta += turningAngle
//                print  "d"

}
void moveRover(){
// main moving logic
    wasAtWaypoint = atWayPoint();
    Turn();
//    startingPointWorldCoordinate = copy.copy(roverWorldCoordinate)
}

int main()
{
    iniatilze(); // set a random destation
    string line; //strores the raw string of the NEDS example of a line -73.6010,67.6970,0.0000
    ifstream myfile ("log.txt"); // use example a log given from piksi gps
    std::vector<std::string> cordStr; // a vector collection of the NED
     if (myfile.is_open()) // make sure the file exist
    {
    while ( getline (myfile,line) ) // get a line from the log
    {
      cordStr.clear(); // clear the contents of the vector so it doesnt' grow from last line
       split( line, ',', cordStr); // we parse each coordinate of the NED ito a list of strings for each ned
//       for (std::vector<string>::iterator it = cordStr.begin(); it != cordStr.end(); ++it)
//            std::cout << ' ' << *it; // iterate through vector of coordinates and print them
        if (lineNumber > 2) // we skip the first two lines because they are errors from piksi
        {
            try // we try to convert the coordinates from strings to floats
            {

                gpsX = std::stof (cordStr[1], &sz);
                gpsY = std::stof (cordStr[0], &sz);
                //cout << gpsX << " " << gpsY << "\n";
            }
            catch(...){ // if we are given errors from piksi we will just go to next line
               // #print "bad coordinates"
                continue;
            }
        }
        if (lineNumber == 3)
        { //after a points been recored you can mark the starting postion
            startingPointWorldCoordinate = Point(gpsX, gpsY);
        }
        if (lineNumber == 4)
        { // next we start tracking two postion in rover postions to determin heading update rovers postion
            roverWorldCoordinate = Point(gpsX, gpsY);
            roverPostions.push_back(roverWorldCoordinate);
            roverPostions.push_back(roverWorldCoordinate);
        }
        if (lineNumber > 4)
        {// only want two rover postions so trim pop off the oldest postion after two.
            roverWorldCoordinate = Point(gpsX, gpsY);
            roverPostions.push_back(roverWorldCoordinate);
            if (roverPostions.size() > 2)
            {
                roverPostions.pop_front();
            }
            moveRover();
        }
        lineNumber += 1;


    }
    myfile.close();
    }

    else cout << "Unable to open file";

    return 0;
}
