//Please, if you use this, share the improvements

using System;

namespace AgOpenGPS
{
    /// <summary>
    /// Represents a three dimensional vector.
    /// </summary>
    ///

    public class vec3 : vec2
    {
        public double heading;

        public vec3(double _easting, double _northing, double _heading) : base (_easting, _northing)
        {
            heading = _heading;
        }

        public vec3(vec3 v) : base(v.easting, v.northing)
        {
            heading = v.heading;
        }
    }

    public class CRecPathPt : vec3
    {
        public double speed = 0;
        public bool autoBtnState;

        //constructor
        public CRecPathPt(double _easting, double _northing, double _heading, double _speed, bool _autoBtnState) : base(_easting, _northing, _heading)
        {
            speed = _speed;
            autoBtnState = _autoBtnState;
        }
    }

    public class vecFix2Fix : vec2
    {
        public double distance; //distance since last point
        public int isSet;    //altitude

        public vecFix2Fix(double _easting, double _northing, double _distance, int _isSet) : base(_easting, _northing)
        {
            distance = _distance;
            isSet = _isSet;
        }
    }

    public class vec6 : vec2
    {
        public int crossingIdx;
        public int turnLineIdx;
        public int boundaryIndex;
        public double time;

        public vec6(double _easting, double _northing, double _time, int _boundaryindex, int _crosssingidx, int _turnlineidx) : base(_easting, _northing)
        {
            time = _time;
            boundaryIndex = _boundaryindex;
            crossingIdx = _crosssingidx;
            turnLineIdx = _turnlineidx;
        }
    }

    public class vec2
    {
        public double easting; //easting
        public double northing; //northing

        public vec2(vec2 v)
        {
            easting = v.easting;
            northing = v.northing;
        }

        public vec2(double _easting, double _northing)
        {
            easting = _easting;
            northing = _northing;
        }

        public static vec2 operator -(vec2 lhs, vec2 rhs)
        {
            return new vec2(lhs.easting - rhs.easting, lhs.northing - rhs.northing);
        }

        //calculate the heading of dirction pointx to pointz
        public double HeadingXZ()
        {
            return Math.Atan2(easting, northing);
        }

        //normalize to 1
        public vec2 Normalize()
        {
            double length = GetLength();
            if (Math.Abs(length) < 0.000000000001)
            {
                throw new DivideByZeroException("Trying to normalize a vector with length of zero.");
            }

            return new vec2(easting /= length, northing /= length);
        }

        //Returns the length of the vector
        public double GetLength()
        {
            return Math.Sqrt((easting * easting) + (northing * northing));
        }

        // Calculates the squared length of the vector.
        public double GetLengthSquared()
        {
            return (easting * easting) + (northing * northing);
        }

        //scalar double
        public static vec2 operator *(vec2 self, double s)
        {
            return new vec2(self.easting * s, self.northing * s);
        }

        //add 2 vectors
        public static vec2 operator +(vec2 lhs, vec2 rhs)
        {
            return new vec2(lhs.easting + rhs.easting, lhs.northing + rhs.northing);
        }
    }
}