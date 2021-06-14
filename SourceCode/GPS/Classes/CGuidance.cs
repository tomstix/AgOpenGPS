using System;
using System.Collections.Generic;

namespace AgOpenGPS
{
    public class CGuidance
    {
        private readonly FormGPS mf;

        //steer, pivot, and ref indexes
        private int sA, sB, pA, pB;

        public double distanceFromCurrentLineSteer, distanceFromCurrentLinePivot;
        public double steerAngleGu;

        public double inty, xTrackSteerCorrection = 0;
        public double steerHeadingError, steerHeadingErrorDegrees;

        public double distSteerError, lastDistSteerError, derivativeDistError;

        public double pivotDistanceError;

        //derivative counter
        private int counter;

        public CGuidance(FormGPS _f)
        {
            //constructor
            mf = _f;
        }

        private void DoSteerAngleCalc()
        {
            if (mf.isReverse) steerHeadingError *= -1;
            //Overshoot setting on Stanley tab
            steerHeadingError *= mf.vehicle.stanleyHeadingErrorGain;

            double sped = Math.Abs(mf.avgSpeed);
            if (sped > 1) sped = 1 + 0.277 * (sped - 1);
            else sped = 1;
            double XTEc = Math.Atan((distanceFromCurrentLineSteer * mf.vehicle.stanleyDistanceErrorGain)
                / (sped));

            xTrackSteerCorrection = (xTrackSteerCorrection * 0.5) + XTEc * (0.5);

            steerAngleGu = glm.toDegrees((xTrackSteerCorrection + steerHeadingError) * -1.0);

            if (Math.Abs(distanceFromCurrentLineSteer) > 0.5) steerAngleGu *= 0.5;
            else steerAngleGu *= (1 - Math.Abs(distanceFromCurrentLineSteer));

            if (steerAngleGu < -mf.vehicle.maxSteerAngle) steerAngleGu = -mf.vehicle.maxSteerAngle;
            else if (steerAngleGu > mf.vehicle.maxSteerAngle) steerAngleGu = mf.vehicle.maxSteerAngle;

            //integral slider is set to 0
            if (mf.vehicle.stanleyIntegralGainAB != 0 || mf.isReverse)
            {
                //derivative of steer distance error
                distSteerError = (distSteerError * 0.95) + ((xTrackSteerCorrection * 60) * 0.05);
                if (counter++ > 5)
                {
                    derivativeDistError = distSteerError - lastDistSteerError;
                    lastDistSteerError = distSteerError;
                    counter = 0;
                }

                pivotDistanceError = (pivotDistanceError * 0.6) + (distanceFromCurrentLinePivot * 0.4);

                if (mf.pn.speed > mf.startSpeed
                    && mf.isAutoSteerBtnOn
                    && Math.Abs(derivativeDistError) < 1
                    && Math.Abs(pivotDistanceError) < 0.25)
                {
                    //if over the line heading wrong way, rapidly decrease integral
                    if ((inty < 0 && distanceFromCurrentLinePivot < 0) || (inty > 0 && distanceFromCurrentLinePivot > 0))
                    {
                        inty += pivotDistanceError * mf.vehicle.stanleyIntegralGainAB * -0.1;
                    }
                    else
                    {
                        inty += pivotDistanceError * mf.vehicle.stanleyIntegralGainAB * -0.01;
                    }

                    if (inty > 5.0) inty = 5.0;
                    else if (inty < -5.0) inty = -5.0;
                }
                else inty *= 0.7;
            }
            else inty = 0;

            //Convert to millimeters from meters
            mf.guidanceLineDistanceOff = (short)Math.Round(distanceFromCurrentLinePivot * 1000.0, MidpointRounding.AwayFromZero);
            mf.guidanceLineSteerAngle = (short)(steerAngleGu * 100);
        }

        /// <summary>
        /// Function to calculate steer angle for AB Line Segment only
        /// No curvature calc on straight line
        /// </summary>
        /// <param name="curPtA"></param>
        /// <param name="curPtB"></param>
        /// <param name="pivot"></param>
        /// <param name="steer"></param>
        /// <param name="isValid"></param>
        public void StanleyGuidanceABLine(vec3 curPtA, vec3 curPtB, vec3 pivot, vec3 steer)
        {
            //get the pivot distance from currently active AB segment   ///////////  Pivot  ////////////
            double dx = curPtB.easting - curPtA.easting;
            double dy = curPtB.northing - curPtA.northing;
            if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dy) < Double.Epsilon) return;

            //how far from current AB Line is fix
            distanceFromCurrentLinePivot = ((dy * pivot.easting) - (dx * pivot.northing) + (curPtB.easting
                        * curPtA.northing) - (curPtB.northing * curPtA.easting))
                            / Math.Sqrt((dy * dy) + (dx * dx));

            if (!mf.ABLine.isHeadingSameWay)
                distanceFromCurrentLinePivot *= -1.0;

            mf.ABLine.distanceFromCurrentLinePivot = distanceFromCurrentLinePivot;
            double U = (((pivot.easting - curPtA.easting) * dx)
                            + ((pivot.northing - curPtA.northing) * dy))
                            / ((dx * dx) + (dy * dy));

            mf.ABLine.rEastAB = curPtA.easting + (U * dx);
            mf.ABLine.rNorthAB = curPtA.northing + (U * dy);

            //get the distance from currently active AB segment of steer axle //////// steer /////////////

            dx = curPtB.easting - curPtA.easting;
            dy = curPtB.northing - curPtA.northing;

            if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dy) < Double.Epsilon) return;

            //how far from current AB Line is fix
            distanceFromCurrentLineSteer = ((dy * steer.easting) - (dx * steer.northing) + (curPtB.easting
                        * curPtA.northing) - (curPtB.northing * curPtA.easting))
                            / Math.Sqrt((dy * dy) + (dx * dx));

            if (!mf.ABLine.isHeadingSameWay)
                distanceFromCurrentLineSteer *= -1.0;

            //create the AB segment to offset
            if (inty != 0)
                distanceFromCurrentLineSteer -= inty;

            steerHeadingError = (steer.heading - curPtA.heading);
            //Fix the circular error
            while (steerHeadingError > glm.PIBy2)
                steerHeadingError -= Math.PI;
            while (steerHeadingError < -glm.PIBy2)
                steerHeadingError += Math.PI;

            DoSteerAngleCalc();
        }


        /// <summary>
        /// Find the steer angle for a curve list, curvature and integral
        /// </summary>
        /// <param name="pivot">Pivot position vector</param>
        /// <param name="steer">Steer position vector</param>
        /// <param name="curList">the current list of guidance points</param>
        public void StanleyGuidanceCurve(vec3 pivot, vec3 steer, ref List<vec3> curList)
        {
            //find the closest point roughly
            int cc = 0, dd;
            int ptCount = curList.Count;
            if (ptCount > 5)
            {
                double minDistA = 1000000, minDistB;

                for (int j = 0; j < ptCount; j += 10)
                {
                    double dist = ((steer.easting - curList[j].easting) * (steer.easting - curList[j].easting))
                                    + ((steer.northing - curList[j].northing) * (steer.northing - curList[j].northing));
                    if (dist < minDistA)
                    {
                        minDistA = dist;
                        cc = j;
                    }
                }

                minDistA = minDistB = 1000000;
                dd = cc + 7; if (dd > ptCount - 1) dd = ptCount;
                cc -= 7; if (cc < 0) cc = 0;

                //find the closest 2 points to current close call
                for (int j = cc; j < dd; j++)
                {
                    double dist = ((steer.easting - curList[j].easting) * (steer.easting - curList[j].easting))
                                    + ((steer.northing - curList[j].northing) * (steer.northing - curList[j].northing));
                    if (dist < minDistA)
                    {
                        minDistB = minDistA;
                        sB = sA;
                        minDistA = dist;
                        sA = j;
                    }
                    else if (dist < minDistB)
                    {
                        minDistB = dist;
                        sB = j;
                    }
                }

                //just need to make sure the points continue ascending or heading switches all over the place
                if (sA > sB) { int C = sA; sA = sB; sB = C; }

                //currentLocationIndex = sA;
                if (sA > ptCount - 1 || sB > ptCount - 1) return;

                minDistA = minDistB = 1000000;

                if (mf.curve.isHeadingSameWay)
                {
                    dd = sB; cc = dd - 12; if (cc < 0) cc = 0;
                }
                else
                {
                    cc = sA; dd = sA + 12; if (dd >= ptCount) dd = ptCount - 1;
                }

                //find the closest 2 points of pivot back from steer
                for (int j = cc; j < dd; j++)
                {
                    double dist = ((pivot.easting - curList[j].easting) * (pivot.easting - curList[j].easting))
                                    + ((pivot.northing - curList[j].northing) * (pivot.northing - curList[j].northing));
                    if (dist < minDistA)
                    {
                        minDistB = minDistA;
                        pB = pA;
                        minDistA = dist;
                        pA = j;
                    }
                    else if (dist < minDistB)
                    {
                        minDistB = dist;
                        pB = j;
                    }
                }

                //just need to make sure the points continue ascending or heading switches all over the place
                if (pA > pB) { int C = pA; pA = pB; pB = C; }

                if (pA > ptCount - 1 || pB > ptCount - 1)
                {
                    pA = ptCount - 2;
                    pB = ptCount - 1;
                }

                //get the pivot distance from currently active AB segment   ///////////  Pivot  ////////////
                double dx = curList[pB].easting - curList[pA].easting;
                double dy = curList[pB].northing - curList[pA].northing;

                if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dy) < Double.Epsilon) return;

                //how far from current AB Line is fix
                distanceFromCurrentLinePivot = ((dy * pivot.easting) - (dx * pivot.northing) + (curList[pB].easting
                            * curList[pA].northing) - (curList[pB].northing * curList[pA].easting))
                                / Math.Sqrt((dy * dy) + (dx * dx));

                if (!mf.curve.isHeadingSameWay)
                    distanceFromCurrentLinePivot *= -1.0;

                double U = (((pivot.easting - curList[pA].easting) * dx)
                                + ((pivot.northing - curList[pA].northing) * dy))
                                / ((dx * dx) + (dy * dy));

                mf.curve.manualUturnHeading = curList[pA].heading;
                mf.curve.rEastCu = curList[pA].easting + (U * dx);
                mf.curve.rNorthCu = curList[pA].northing + (U * dy);
                mf.curve.currentLocationIndex = pA;
                mf.curve.distanceFromCurrentLinePivot = distanceFromCurrentLinePivot;

                ////////// steer ///////////// get the distance from currently active AB segment of steer axle 
                dx = curList[sB].easting - curList[sA].easting;
                dy = curList[sB].northing - curList[sA].northing;

                if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dy) < Double.Epsilon) return;

                //how far from current AB Line is fix
                distanceFromCurrentLineSteer = ((dy * steer.easting) - (dx * steer.northing) + (curList[sB].easting
                            * curList[sA].northing) - (curList[sB].northing * curList[sA].easting))
                                / Math.Sqrt((dy * dy) + (dx * dx));

                if (!mf.curve.isHeadingSameWay)
                    distanceFromCurrentLineSteer *= -1.0;
                //create the integral offset
                if (inty != 0)
                    distanceFromCurrentLineSteer -= inty;

                steerHeadingError = steer.heading - curList[sB].heading;

                //Fix the circular error
                while (steerHeadingError > glm.PIBy2)
                    steerHeadingError -= Math.PI;
                while (steerHeadingError < -glm.PIBy2)
                    steerHeadingError += Math.PI;

                DoSteerAngleCalc();
            }
            else
            {
                //invalid distance so tell AS module
                distanceFromCurrentLineSteer = 32000;
                mf.guidanceLineDistanceOff = 32000;
            }
        }
    
        public void PurePursuitGuidance(vec3 currentABLineP1, vec3 currentABLineP2, vec3 pivot, vec3 steer)
        {
            //get the distance from currently active AB line
            //x2-x1
            double dx = currentABLineP2.easting - currentABLineP1.easting;
            //z2-z1
            double dy = currentABLineP2.northing - currentABLineP1.northing;

            double length = Math.Sqrt((dy * dy) + (dx * dx));
            //how far from current AB Line is fix
            distanceFromCurrentLinePivot = ((dy * pivot.easting) - (dx * pivot.northing) + (currentABLineP2.easting
                        * currentABLineP1.northing) - (currentABLineP2.northing * currentABLineP1.easting))
                        / length;

            //distance is negative if on left, positive if on right
            if (!mf.ABLine.isHeadingSameWay)
                distanceFromCurrentLinePivot *= -1.0;

            //integral slider is set to 0
            if (mf.vehicle.purePursuitIntegralGain != 0 || mf.isReverse)
            {
                pivotDistanceError = pivotDistanceError * 0.8 + distanceFromCurrentLinePivot * 0.2;

                if (counter++ > 4)
                {
                    derivativeDistError = pivotDistanceError - lastDistSteerError;
                    lastDistSteerError = pivotDistanceError;
                    counter = 0;
                    derivativeDistError *= 2;
                }

                if (mf.isAutoSteerBtnOn && mf.avgSpeed > 2.5 && Math.Abs(derivativeDistError) < 0.1)
                {
                    //if over the line heading wrong way, rapidly decrease integral
                    if ((inty < 0 && distanceFromCurrentLinePivot < 0) || (inty > 0 && distanceFromCurrentLinePivot > 0))
                    {
                        inty += pivotDistanceError * mf.vehicle.purePursuitIntegralGain * -0.04;
                    }
                    else
                        inty += pivotDistanceError * mf.vehicle.purePursuitIntegralGain * -0.02;

                    if (inty > 5.0) inty = 5.0;
                    else if (inty < -5.0) inty = -5.0;
                }
                else inty *= 0.96;
            }
            else inty = 0;

            // ** Pure pursuit ** - calc point on ABLine closest to current position
            double U = (((pivot.easting - currentABLineP1.easting) * dx)
                        + ((pivot.northing - currentABLineP1.northing) * dy))
                        / ((dx * dx) + (dy * dy));

            //point on AB line closest to pivot axle point
            mf.ABLine.rEastAB = currentABLineP1.easting + (U * dx);
            mf.ABLine.rNorthAB = currentABLineP1.northing + (U * dy);

            //update base on autosteer settings and distance from line
            double goalPointDistance = mf.vehicle.UpdateGoalPointDistance();

            if (mf.isReverse ? !mf.ABLine.isHeadingSameWay : mf.ABLine.isHeadingSameWay)
            {
                mf.ABLine.goalPointAB.easting = mf.ABLine.rEastAB + (Math.Sin(mf.ABLine.abHeading) * goalPointDistance) + inty * (dy / length);
                mf.ABLine.goalPointAB.northing = mf.ABLine.rNorthAB + (Math.Cos(mf.ABLine.abHeading) * goalPointDistance) - inty * (dx / length);
            }
            else
            {
                mf.ABLine.goalPointAB.easting = mf.ABLine.rEastAB - (Math.Sin(mf.ABLine.abHeading) * goalPointDistance) - inty * (dy / length);
                mf.ABLine.goalPointAB.northing = mf.ABLine.rNorthAB - (Math.Cos(mf.ABLine.abHeading) * goalPointDistance) + inty * (dx / length);
            }

            //calc "D" the distance from pivot axle to lookahead point
            double goalPointDistanceDSquared
                = glm.DistanceSquared(mf.ABLine.goalPointAB.northing, mf.ABLine.goalPointAB.easting, pivot.northing, pivot.easting);

            //calculate the the new x in local coordinates and steering angle degrees based on wheelbase
            double localHeading = glm.twoPI - mf.fixHeading;

            mf.ABLine.ppRadiusAB = goalPointDistanceDSquared / (2 * (((mf.ABLine.goalPointAB.easting - pivot.easting) * Math.Cos(localHeading))
                + ((mf.ABLine.goalPointAB.northing - pivot.northing) * Math.Sin(localHeading))));

            mf.ABLine.steerAngleAB = glm.toDegrees(Math.Atan(2 * (((mf.ABLine.goalPointAB.easting - pivot.easting) * Math.Cos(localHeading))
                + ((mf.ABLine.goalPointAB.northing - pivot.northing) * Math.Sin(localHeading))) * mf.vehicle.wheelbase
                / goalPointDistanceDSquared));
            if (mf.ABLine.steerAngleAB < -mf.vehicle.maxSteerAngle) mf.ABLine.steerAngleAB = -mf.vehicle.maxSteerAngle;
            if (mf.ABLine.steerAngleAB > mf.vehicle.maxSteerAngle) mf.ABLine.steerAngleAB = mf.vehicle.maxSteerAngle;

            //limit circle size for display purpose
            if (mf.ABLine.ppRadiusAB < -500) mf.ABLine.ppRadiusAB = -500;
            if (mf.ABLine.ppRadiusAB > 500) mf.ABLine.ppRadiusAB = 500;

            mf.ABLine.radiusPointAB.easting = pivot.easting + (mf.ABLine.ppRadiusAB * Math.Cos(localHeading));
            mf.ABLine.radiusPointAB.northing = pivot.northing + (mf.ABLine.ppRadiusAB * Math.Sin(localHeading));

            if (mf.isAngVelGuidance)
            {
                //angular velocity in rads/sec  = 2PI * m/sec * radians/meters
                mf.setAngVel = 0.277777 * mf.pn.speed * (Math.Tan(glm.toRadians(mf.ABLine.steerAngleAB))) / mf.vehicle.wheelbase;
                mf.setAngVel = glm.toDegrees(mf.setAngVel) * 100;

                //clamp the steering angle to not exceed safe angular velocity
                if (Math.Abs(mf.setAngVel) > 1000)
                {
                    //mf.setAngVel = mf.setAngVel < 0 ? -mf.vehicle.maxAngularVelocity : mf.vehicle.maxAngularVelocity;
                    mf.setAngVel = mf.setAngVel < 0 ? -1000 : 1000;
                }
            }

            //Convert to millimeters
            mf.guidanceLineDistanceOff = (short)Math.Round(distanceFromCurrentLinePivot * 1000.0, MidpointRounding.AwayFromZero);
            mf.guidanceLineSteerAngle = (short)(mf.ABLine.steerAngleAB * 100);
        }

        public void PurePursuitGuidanceCurve(vec3 pivot, ref List<vec3> curList)
        {
            double dist, dx, dy;
            double minDistA = double.MaxValue, minDistB = double.MaxValue;

            //find the closest 2 points to current fix
            for (int t = 0; t < curList.Count; t++)
            {
                dist = glm.DistanceSquared(pivot, curList[t]);

                if (dist < minDistA)
                {
                    minDistB = minDistA;
                    pB = pA;
                    minDistA = dist;
                    pA = t;
                }
                else if (dist < minDistB)
                {
                    minDistB = dist;
                    pB = t;
                }
            }

            //just need to make sure the points continue ascending or heading switches all over the place
            if (pA > pB) { int C = pA; pA = pB; pB = C; }

            //get the distance from currently active AB line
            dx = curList[pB].easting - curList[pA].easting;
            dy = curList[pB].northing - curList[pA].northing;

            if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dy) < Double.Epsilon) return;

            //abHeading = Math.Atan2(dz, dx);
            double length = Math.Sqrt((dy * dy) + (dx * dx));
            //how far from current AB Line is fix
            distanceFromCurrentLinePivot = ((dy * pivot.easting) - (dx * pivot.northing) + (curList[pB].easting
                        * curList[pA].northing) - (curList[pB].northing * curList[pA].easting))
                            / length;

            if (mf.curve.isHeadingSameWay)
                distanceFromCurrentLinePivot *= -1.0;

            //integral slider is set to 0
            if (mf.vehicle.purePursuitIntegralGain != 0 || mf.isReverse)
            {
                pivotDistanceError = pivotDistanceError * 0.8 + distanceFromCurrentLinePivot * 0.2;

                if (counter++ > 4)
                {
                    derivativeDistError = pivotDistanceError - lastDistSteerError;
                    lastDistSteerError = pivotDistanceError;
                    counter = 0;
                    derivativeDistError *= 2;
                }

                if (mf.isAutoSteerBtnOn && mf.avgSpeed > 2.5 && Math.Abs(derivativeDistError) < 0.1)
                {
                    //if over the line heading wrong way, rapidly decrease integral
                    if ((inty < 0 && distanceFromCurrentLinePivot < 0) || (inty > 0 && distanceFromCurrentLinePivot > 0))
                    {
                        inty += pivotDistanceError * mf.vehicle.purePursuitIntegralGain * -0.04;
                    }
                    else
                        inty += pivotDistanceError * mf.vehicle.purePursuitIntegralGain * -0.02;

                    if (inty > 5.0) inty = 5.0;
                    else if (inty < -5.0) inty = -5.0;
                }
                else inty *= 0.96;
            }
            else inty = 0;
            
            // ** Pure pursuit ** - calc point on ABLine closest to current position
            double U = (((pivot.easting - curList[pA].easting) * dx)
                        + ((pivot.northing - curList[pA].northing) * dy))
                        / ((dx * dx) + (dy * dy));

            mf.curve.manualUturnHeading = curList[pA].heading;
            mf.curve.rEastCu = curList[pA].easting + (U * dx);
            mf.curve.rNorthCu = curList[pA].northing + (U * dy);
            mf.curve.currentLocationIndex = pA;
            mf.curve.distanceFromCurrentLinePivot = distanceFromCurrentLinePivot;

            //update base on autosteer settings and distance from line
            double goalPointDistance = mf.vehicle.UpdateGoalPointDistance();

            bool ReverseHeading = mf.isReverse ? !mf.curve.isHeadingSameWay : mf.curve.isHeadingSameWay;

            int count = ReverseHeading ? 1 : -1;
            vec3 start = new vec3(mf.curve.rEastCu, mf.curve.rNorthCu, 0);
            double distSoFar = 0;

            for (int i = ReverseHeading ? pB : pA; i < curList.Count && i >= 0; i += count)
            {
                // used for calculating the length squared of next segment.
                double tempDist = glm.Distance(start, curList[i]);

                //will we go too far?
                if ((tempDist + distSoFar) > goalPointDistance)
                {
                    double j = (goalPointDistance - distSoFar) / tempDist; // the remainder to yet travel

                    mf.curve.goalPointCu.easting = (((1 - j) * start.easting) + (j * curList[i].easting));
                    mf.curve.goalPointCu.northing = (((1 - j) * start.northing) + (j * curList[i].northing));
                    break;
                }
                else distSoFar += tempDist;
                start = curList[i];
            }

            //create the integral offset
            mf.curve.goalPointCu.easting += ReverseHeading ? inty * (dy / length) : inty * -(dy / length);
            mf.curve.goalPointCu.northing += ReverseHeading ? inty * -(dx / length) : inty * (dx / length);

            //calc "D" the distance from pivot axle to lookahead point
            double goalPointDistanceSquared = glm.DistanceSquared(mf.curve.goalPointCu.northing, mf.curve.goalPointCu.easting, pivot.northing, pivot.easting);

            double localHeading = glm.twoPI - mf.fixHeading;

            mf.curve.ppRadiusCu = goalPointDistanceSquared / (2 * (((mf.curve.goalPointCu.easting - pivot.easting) * Math.Cos(localHeading)) + ((mf.curve.goalPointCu.northing - pivot.northing) * Math.Sin(localHeading))));

            mf.curve.steerAngleCu = glm.toDegrees(Math.Atan(2 * (((mf.curve.goalPointCu.easting - pivot.easting) * Math.Cos(localHeading))
                + ((mf.curve.goalPointCu.northing - pivot.northing) * Math.Sin(localHeading))) * mf.vehicle.wheelbase / goalPointDistanceSquared));

            if (mf.curve.steerAngleCu < -mf.vehicle.maxSteerAngle) mf.curve.steerAngleCu = -mf.vehicle.maxSteerAngle;
            if (mf.curve.steerAngleCu > mf.vehicle.maxSteerAngle) mf.curve.steerAngleCu = mf.vehicle.maxSteerAngle;

            if (mf.curve.ppRadiusCu < -500) mf.curve.ppRadiusCu = -500;
            if (mf.curve.ppRadiusCu > 500) mf.curve.ppRadiusCu = 500;

            mf.curve.radiusPointCu.easting = pivot.easting + (mf.curve.ppRadiusCu * Math.Cos(localHeading));
            mf.curve.radiusPointCu.northing = pivot.northing + (mf.curve.ppRadiusCu * Math.Sin(localHeading));

            //angular velocity in rads/sec  = 2PI * m/sec * radians/meters
            double angVel = glm.twoPI * 0.277777 * mf.pn.speed * (Math.Tan(glm.toRadians(mf.curve.steerAngleCu))) / mf.vehicle.wheelbase;

            //clamp the steering angle to not exceed safe angular velocity
            if (Math.Abs(angVel) > mf.vehicle.maxAngularVelocity)
            {
                mf.curve.steerAngleCu = glm.toDegrees(mf.curve.steerAngleCu > 0 ?
                        (Math.Atan((mf.vehicle.wheelbase * mf.vehicle.maxAngularVelocity) / (glm.twoPI * mf.avgSpeed * 0.277777)))
                    : (Math.Atan((mf.vehicle.wheelbase * -mf.vehicle.maxAngularVelocity) / (glm.twoPI * mf.avgSpeed * 0.277777))));
            }

            //Convert to centimeters
            mf.guidanceLineDistanceOff = (short)Math.Round(distanceFromCurrentLinePivot * 1000.0, MidpointRounding.AwayFromZero);
            mf.guidanceLineSteerAngle = (short)(mf.curve.steerAngleCu * 100);
        }
    }
}