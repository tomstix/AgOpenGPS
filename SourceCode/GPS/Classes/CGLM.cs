using System;
using System.Collections.Generic;
using System.Threading;
using System.Windows.Forms;

namespace AgOpenGPS
{
    public static class DanielP
    {
        public static bool GetLineIntersection(vec2 PointAA, vec2 PointAB, vec2 PointBA, vec2 PointBB, out vec2 Crossing, out double TimeA, bool Limit = false)
        {
            TimeA = -1;
            Crossing = new vec2(0.0, 0.0);
            double denominator = (PointAB.northing - PointAA.northing) * (PointBB.easting - PointBA.easting) - (PointBB.northing - PointBA.northing) * (PointAB.easting - PointAA.easting);

            if (denominator != 0.0)
            {
                TimeA = ((PointBB.northing - PointBA.northing) * (PointAA.easting - PointBA.easting) - (PointAA.northing - PointBA.northing) * (PointBB.easting - PointBA.easting)) / denominator;

                if (Limit || (TimeA > 0.0 && TimeA < 1.0))
                {
                    double TimeB = ((PointAB.northing - PointAA.northing) * (PointAA.easting - PointBA.easting) - (PointAA.northing - PointBA.northing) * (PointAB.easting - PointAA.easting)) / denominator;
                    if (Limit || (TimeB > 0.0 && TimeB < 1.0))
                    {
                        Crossing = PointAA + (PointAB - PointAA) * TimeA;
                        return true;
                    }
                    else return false;
                }
                else return false;
            }
            else return false;
        }

        public static int Clamp(this int Idx, int Size)
        {
            return (Size + Idx) % Size;
        }

        public static double GetLength(double dx, double dy)
        {
            return Math.Sqrt(dx * dx + dy * dy);
        }

        public static vec2 GetProportionPoint(this vec2 point, double segment, double length, double dx, double dy)
        {
            double factor = segment / length;
            return new vec2(point.easting - dy * factor, point.northing - dx * factor);
        }

        public static bool CalculateRoundedCorner(this List<vec2> Points, double Radius, double Radius2, bool Loop, double MaxAngle, CancellationToken ct, int Count1 = 0, int Count2 = int.MaxValue)
        {
            double tt = Math.Min(Math.Asin(0.5 / Radius), Math.Asin(0.5 / Radius2));
            if (!double.IsNaN(tt)) MaxAngle = Math.Min(tt, MaxAngle);

            vec2 tt3;
            int A, B, C, oldA, OldC;
            bool reverse = false;

            for (B = Count1; B < Points.Count && B < Count2; B++)
            {
                if (ct.IsCancellationRequested) break;
                if (!Loop && (B == 0 || B + 1 == Points.Count)) continue;
                A = (B == 0) ? Points.Count - 1 : B - 1;
                C = (B + 1 == Points.Count) ? 0 : B + 1;
                bool stop = false;
                double dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0, angle, segment = 0, length1 = 0, length2 = 0;
                while (true)
                {
                    tt3 = Points[B];
                    if (ct.IsCancellationRequested) break;
                    if (GetLineIntersection(Points[A], Points[(A + 1).Clamp(Points.Count)], Points[C], Points[(C - 1).Clamp(Points.Count)], out vec2 Crossing, out double Time, true))
                    {
                        if (Time > -100 && Time < 100)
                            tt3 = Crossing;
                    }

                    dx1 = tt3.northing - Points[A].northing;
                    dy1 = tt3.easting - Points[A].easting;
                    dx2 = tt3.northing - Points[C].northing;
                    dy2 = tt3.easting - Points[C].easting;

                    angle = Math.Atan2(dy1, dx1) - Math.Atan2(dy2, dx2);

                    if (angle < 0) angle += glm.twoPI;
                    if (angle > glm.twoPI) angle -= glm.twoPI;
                    angle /= 2;

                    double Tan = Math.Abs(Math.Tan(angle));

                    angle = Math.Abs(angle);
                    reverse = angle > glm.PIBy2;
                    segment = (reverse ? Radius2 : Radius) / Tan;

                    length1 = GetLength(dx1, dy1);
                    length2 = GetLength(dx2, dy2);
                    oldA = A;
                    OldC = C;
                    if (segment > length1)
                    {
                        if (Loop || (!Loop && A > 0)) A = (A == 0) ? Points.Count - 1 : A - 1;

                        if (A == C)
                        {
                            stop = true;
                            break;
                        }
                    }
                    if (segment > length2)
                    {
                        if (Loop || (!Loop && C < Points.Count - 1)) C = (C + 1 == Points.Count) ? 0 : C + 1;
                        if (C == A)
                        {
                            stop = true;
                            break;
                        }
                    }
                    else if (segment < length1)
                    {
                        Points[B] = tt3;
                        break;
                    }

                    if (oldA == A && OldC == C)
                    {
                        stop = true;
                        break;
                    }
                }
                if (ct.IsCancellationRequested) break;
                if (stop) return false;

                bool Looping = (A > C);
                while (C - 1 > A || Looping)
                {
                    if (ct.IsCancellationRequested) break;
                    if (C == 0)
                    {
                        if (A == Points.Count - 1) break;
                        Looping = false;
                    }

                    C = C == 0 ? Points.Count - 1 : C - 1;

                    if (A > C) A--;

                    Points.RemoveAt(C);
                    if (Count2 != int.MaxValue)
                        Count2--;
                }

                B = A > B ? -1 : A;

                vec2 p1Cross = tt3.GetProportionPoint(segment, length1, dx1, dy1);
                vec2 p2Cross = tt3.GetProportionPoint(segment, length2, dx2, dy2);

                if (reverse)
                {
                    vec2 test = p1Cross;
                    p1Cross = p2Cross;
                    p2Cross = test;
                }

                double dx = tt3.northing * 2 - p1Cross.northing - p2Cross.northing;
                double dy = tt3.easting * 2 - p1Cross.easting - p2Cross.easting;
                if (dx1 == 0 && dy1 == 0 || dx2 == 0 && dy2 == 0 || dx == 0 && dy == 0) continue;

                vec2 circlePoint;

                double L = GetLength(dx, dy);
                double d = GetLength(segment, reverse ? Radius2 : Radius);

                circlePoint = tt3.GetProportionPoint(d, L, dx, dy);

                double startAngle = Math.Atan2(p1Cross.easting - circlePoint.easting, p1Cross.northing - circlePoint.northing);
                double endAngle = Math.Atan2(p2Cross.easting - circlePoint.easting, p2Cross.northing - circlePoint.northing);

                if (startAngle < 0) startAngle += glm.twoPI;
                if (endAngle < 0) endAngle += glm.twoPI;


                double sweepAngle;

                if (((glm.twoPI - endAngle + startAngle) % glm.twoPI) < ((glm.twoPI - startAngle + endAngle) % glm.twoPI))
                    sweepAngle = (glm.twoPI - endAngle + startAngle) % glm.twoPI;
                else
                    sweepAngle = (glm.twoPI - startAngle + endAngle) % glm.twoPI;

                int sign = Math.Sign(sweepAngle);

                if (reverse)
                {
                    sign = -sign;
                    startAngle = endAngle;
                }

                int pointsCount = (int)Math.Round(Math.Abs(sweepAngle / MaxAngle));

                double degreeFactor = sweepAngle / pointsCount;

                vec2[] points = new vec2[pointsCount];

                for (int j = 0; j < pointsCount; ++j)
                {
                    if (ct.IsCancellationRequested) break;
                    var pointX = circlePoint.northing + Math.Cos(startAngle + sign * (j + 1) * degreeFactor) * (reverse ? Radius2 : Radius);
                    var pointY = circlePoint.easting + Math.Sin(startAngle + sign * (j + 1) * degreeFactor) * (reverse ? Radius2 : Radius);
                    points[j] = new vec2(pointY, pointX);
                }

                Points.InsertRange(B + 1, points);
                if (Count2 != int.MaxValue)
                    Count2 += pointsCount;
                B += points.Length;
            }
            return true;
        }
    }

    public static class NudChk
    {
        public static bool CheckValue(this NumericUpDown numericUpDown, ref decimal value)
        {
            if (value < numericUpDown.Minimum)
            {
                value = numericUpDown.Minimum;
                MessageBox.Show("Serious Settings Problem with - " + numericUpDown.Name
                    + " \n\rMinimum has been exceeded\n\rDouble check ALL your Settings and \n\rFix it and Resave Vehicle File",
                "Critical Settings Warning",
                MessageBoxButtons.OK,
                MessageBoxIcon.Error);
                return true;
            }
            else if (value > numericUpDown.Maximum)
            {
                value = numericUpDown.Maximum;
                MessageBox.Show("Serious Settings Problem with - " + numericUpDown.Name
                    + " \n\rMaximum has been exceeded\n\rDouble check ALL your Settings and \n\rFix it and Resave Vehicle File",
                "Critical Settings Warning",
                MessageBoxButtons.OK,
                MessageBoxIcon.Error);
                return true;
            }

            //value is ok
            return false;
        }

        public static bool CheckValueCm(this NumericUpDown numericUpDown, ref double value)
        {
            //convert to cm
            value *= 100;
            bool isChanged = false;

            if (value < (double)numericUpDown.Minimum)
            {
                value = (double)numericUpDown.Minimum / 2.4;
                MessageBox.Show("Serious Settings Problem with - " + numericUpDown.Name
                    + " \n\rMinimum has been exceeded\n\rDouble check ALL your Settings and \n\rFix it and Resave Vehicle File",
                "Critical Settings Warning",
                MessageBoxButtons.OK,
                MessageBoxIcon.Error);
                isChanged = true;
            }
            else if (value > (double)numericUpDown.Maximum)
            {
                value = (double)numericUpDown.Maximum / 2.6;
                MessageBox.Show("Serious Settings Problem with - " + numericUpDown.Name
                    + " \n\rMaximum has been exceeded\n\rDouble check ALL your Settings and \n\rFix it and Resave Vehicle File",
                "Critical Settings Warning",
                MessageBoxButtons.OK,
                MessageBoxIcon.Error);
                isChanged = true;
            }

            //revert back to meters
            value *= 0.01;

            //value is ok
            return isChanged;
        }
    }

    public static class glm
    {
        // Catmull Rom interpoint spline calculation
        public static vec3 Catmull(double t, vec3 p0, vec3 p1, vec3 p2, vec3 p3)
        {
            double tt = t * t;
            double ttt = tt * t;

            double q1 = -ttt + 2.0f * tt - t;
            double q2 = 3.0f * ttt - 5.0f * tt + 2.0f;
            double q3 = -3.0f * ttt + 4.0f * tt + t;
            double q4 = ttt - tt;

            double tx = 0.5f * (p0.easting * q1 + p1.easting * q2 + p2.easting * q3 + p3.easting * q4);
            double ty = 0.5f * (p0.northing * q1 + p1.northing * q2 + p2.northing * q3 + p3.northing * q4);

            return new vec3(tx, ty, 0);

            //f(t) = a_3 * t^3 + a_2 * t^2 + a_1 * t + a_0  cubic polynomial
            //vec3 a = 2.0 * p1;
            //vec3 b = p2 - p0;
            //vec3 c = 2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3;
            //vec3 d = -1.0 * p0 + 3.0 * p1 - 3.0 * p2 + p3;

            //return (0.5 * (a + (t * b) + (t * t * c) + (t * t * t * d)));
            //

            //vec2 p0 = new vec2(1, 0);
            //vec2 p1 = new vec2(3, 2);
            //vec2 p2 = new vec2(5, 3);
            //vec2 p3 = new vec2(6, 1);

            //vec2 a = 2.0 * p1;
            //vec2 b = p2 - p0;
            //vec2 c = 2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3;
            //vec2 d = -1.0 * p0 + 3.0 * p1 - 3.0 * p2 + p3;

            //double tt = 0.25;

            //vec2 pos = 0.5 * (a + (tt*b) + (tt * tt * c) + (tt * tt * tt * d));


        }


        // Catmull Rom gradient calculation
        public static double CatmullGradient(double t, vec3 p0, vec3 p1, vec3 p2, vec3 p3)
        {
            double tt = t * t;

            double q1 = -3.0f * tt + 4.0f * t - 1;
            double q2 = 9.0f * tt - 10.0f * t;
            double q3 = -9.0f * tt + 8.0f * t + 1.0f;
            double q4 = 3.0f * tt - 2.0f * t;

            double tx = 0.5f * (p0.easting * q1 + p1.easting * q2 + p2.easting * q3 + p3.easting * q4);
            double ty = 0.5f * (p0.northing * q1 + p1.northing * q2 + p2.northing * q3 + p3.northing * q4);

            return Math.Atan2(tx, ty);

            //f(t) = a_3 * t^3 + a_2 * t^2 + a_1 * t + a_0  cubic polynomial
            //vec3 a = 2.0 * p1;
            //vec3 b = p2 - p0;
            //vec3 c = 2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3;
            //vec3 d = -1.0 * p0 + 3.0 * p1 - 3.0 * p2 + p3;

            //return (0.5 * (a + (t * b) + (t * t * c) + (t * t * t * d)));
            //
        }

        //Regex file expression
        public static string fileRegex = "(^(PRN|AUX|NUL|CON|COM[1-9]|LPT[1-9]|(\\.+)$)(\\..*)?$)|(([\\x00-\\x1f\\\\?*:\";‌​|/<>])+)|([\\.]+)";

        //inches to meters
        public static double in2m = 0.0254;

        //meters to inches
        public static double m2in = 39.3701;

        //meters to feet
        public static double m2ft = 3.28084;

        //feet to meters
        public static double ft2m = 0.3048;

        //Hectare to Acres
        public static double ha2ac = 2.47105;

        //Acres to Hectare
        public static double ac2ha = 0.404686;

        //Meters to Acres
        public static double m2ac = 0.000247105;

        //Meters to Hectare
        public static double m2ha = 0.0001;

        // liters per hectare to us gal per acre
        public static double galAc2Lha = 9.35396;

        //us gal per acre to liters per hectare
        public static double LHa2galAc = 0.106907;

        //Liters to Gallons
        public static double L2Gal = 0.264172;

        //Gallons to Liters
        public static double Gal2L = 3.785412534258;

        //the pi's
        public static double twoPI = 6.28318530717958647692;

        public static double PIBy2 = 1.57079632679489661923;

        //Degrees Radians Conversions
        public static double toDegrees(double radians)
        {
            return radians * 57.295779513082325225835265587528;
        }

        public static double toRadians(double degrees)
        {
            return degrees * 0.01745329251994329576923690768489;
        }

        //Distance calcs of all kinds
        public static double Distance(double east1, double north1, double east2, double north2)
        {
            return Math.Sqrt(
                Math.Pow(east1 - east2, 2)
                + Math.Pow(north1 - north2, 2));
        }

        public static double Distance(vec2 first, vec2 second)
        {
            return Math.Sqrt(
                Math.Pow(first.easting - second.easting, 2)
                + Math.Pow(first.northing - second.northing, 2));
        }

        public static double Distance(vec2 first, vec3 second)
        {
            return Math.Sqrt(
                Math.Pow(first.easting - second.easting, 2)
                + Math.Pow(first.northing - second.northing, 2));
        }

        public static double Distance(vec3 first, vec2 second)
        {
            return Math.Sqrt(
                Math.Pow(first.easting - second.easting, 2)
                + Math.Pow(first.northing - second.northing, 2));
        }

        public static double Distance(vec3 first, vec3 second)
        {
            return Math.Sqrt(
                Math.Pow(first.easting - second.easting, 2)
                + Math.Pow(first.northing - second.northing, 2));
        }

        public static double Distance(vec2 first, double east, double north)
        {
            return Math.Sqrt(
                Math.Pow(first.easting - east, 2)
                + Math.Pow(first.northing - north, 2));
        }

        public static double Distance(vec3 first, double east, double north)
        {
            return Math.Sqrt(
                Math.Pow(first.easting - east, 2)
                + Math.Pow(first.northing - north, 2));
        }

        public static double Distance(vecFix2Fix first, vec2 second)
        {
            return Math.Sqrt(
                Math.Pow(first.easting - second.easting, 2)
                + Math.Pow(first.northing - second.northing, 2));
        }

        public static double Distance(vecFix2Fix first, vecFix2Fix second)
        {
            return Math.Sqrt(
                Math.Pow(first.easting - second.easting, 2)
                + Math.Pow(first.northing - second.northing, 2));
        }



        //not normalized distance, no square root
        public static double DistanceSquared(double northing1, double easting1, double northing2, double easting2)
        {
            return Math.Pow(easting1 - easting2, 2) + Math.Pow(northing1 - northing2, 2);
        }

        public static double DistanceSquared(vec3 first, vec2 second)
        {
            return (
            Math.Pow(first.easting - second.easting, 2)
            + Math.Pow(first.northing - second.northing, 2));
        }

        public static double DistanceSquared(vec2 first, vec3 second)
        {
            return (
            Math.Pow(first.easting - second.easting, 2)
            + Math.Pow(first.northing - second.northing, 2));
        }

        public static double DistanceSquared(vec3 first, vec3 second)
        {
            return (
            Math.Pow(first.easting - second.easting, 2)
            + Math.Pow(first.northing - second.northing, 2));
        }
        public static double DistanceSquared(vec2 first, vec2 second)
        {
            return (
            Math.Pow(first.easting - second.easting, 2)
            + Math.Pow(first.northing - second.northing, 2));
        }
    }
}