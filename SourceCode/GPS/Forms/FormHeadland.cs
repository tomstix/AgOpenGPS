using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;

namespace AgOpenGPS
{
    public partial class FormHeadland : Form
    {
        //access to the main GPS form and all its variables
        private readonly FormGPS mf = null;

        private double maxFieldX, maxFieldY, minFieldX, minFieldY, fieldCenterX, fieldCenterY, maxFieldDistance;
        private Point fixPt;

        private bool isA, isSet;
        public double headWidth = 0;
        private int start = -1, end = -1;
        private double totalHeadlandWidth = 0;

        public vec3 pint = new vec3(0.0, 1.0, 0.0);

        //list of coordinates of boundary line
        public List<vec3> headLineTemplate = new List<vec3>();

        private vec3[] hdArr;

        public FormHeadland(Form callingForm)
        {
            //get copy of the calling main form
            mf = callingForm as FormGPS;

            InitializeComponent();
            //lblPick.Text = gStr.gsSelectALine;
            this.Text = gStr.gsHeadlandForm;
            btnReset.Text = gStr.gsResetAll;

            nudDistance.Controls[0].Enabled = false;
        }

        private void FormHeadland_Load(object sender, EventArgs e)
        {
            isA = true;
            isSet = false;

            lblHeadlandWidth.Text = "0";
            lblWidthUnits.Text = mf.unitsFtM;

            //Builds line
            nudDistance.Value = 0;
            nudSetDistance.Value = 0;

            if (mf.hd.headArr[0].hdLine.Count > 0)
            {
                hdArr = new vec3[mf.hd.headArr[0].hdLine.Count];
                mf.hd.headArr[0].hdLine.CopyTo(hdArr);
                RebuildHeadLineTemplate();
            }
            else
            {
                BuildHeadLineTemplateFromBoundary();
            }
            mf.CloseTopMosts();
        }

        public void BuildHeadLineTemplateFromBoundary()
        {
            totalHeadlandWidth = 0;
            lblHeadlandWidth.Text = "0";
            nudDistance.Value = 0;
            //nudSetDistance.Value = 0;

            //outside boundary - count the points from the boundary
            headLineTemplate.Clear();

            int ptCount = mf.bnd.bndArr[0].bndLine.Count;

            hdArr = new vec3[ptCount];

            for (int i = 0; i < ptCount; i++)
            {
                //calculate the point inside the boundary
                headLineTemplate.Insert(0, new vec3(
                    mf.bnd.bndArr[0].bndLine[i].easting,
                    mf.bnd.bndArr[0].bndLine[i].northing,
                    mf.bnd.bndArr[0].bndLine[i].heading));
            }
            headLineTemplate.CopyTo(hdArr);

            start = end = -1;
            isSet = false;
        }

        private void RebuildHeadLineTemplate()
        {
            headLineTemplate.Clear();


            //Builds line
            nudDistance.Value = 0;
            //nudSetDistance.Value = 0;

            int cnt = hdArr.Length;
            for (int i = 0; i < cnt; i++)
            {
                headLineTemplate.Add(new vec3(hdArr[i]));
            }

            cnt = headLineTemplate.Count;

            hdArr = new vec3[cnt];
            headLineTemplate.CopyTo(hdArr);

            start = end = -1;
            isSet = false;
        }

        private void FixTurnLine(double totalHeadWidth, List<vec3> curBnd, double spacing)
        {
            //count the points from the boundary

            List<vec3> foos = new List<vec3>(hdArr);

            int lineCount = foos.Count;
            double distance;

            //int headCount = mf.bndArr[inTurnNum].bndLine.Count;
            int bndCount = curBnd.Count;
            //remove the points too close to boundary
            for (int i = 0; i < bndCount; i++)
            {
                for (int j = 0; j < lineCount; j++)
                {
                    //make sure distance between headland and boundary is not less then width
                    distance = glm.Distance(curBnd[i], foos[j]);
                    if (distance < (totalHeadWidth * 0.96))
                    {
                        if (j > -1 && j < foos.Count)
                        {
                            foos.RemoveAt(j);
                            lineCount = foos.Count;
                        }
                        j = -1;
                    }
                }
            }

            //make sure distance isn't too small between points on turnLine
            bndCount = foos.Count;

            //double spacing = mf.tool.toolWidth * 0.25;
            for (int i = 0; i < bndCount - 1; i++)
            {
                distance = glm.Distance(foos[i], foos[i + 1]);
                if (distance < spacing)
                {
                    if (i > -1 && (i + 1) < foos.Count)
                    {
                        foos.RemoveAt(i + 1);
                        bndCount = foos.Count;
                    }
                    i--;
                }
            }

            bndCount = foos.Count;

            hdArr = new vec3[bndCount];
            foos.CopyTo(hdArr);
        }

        private void btnSetDistance_Click(object sender, EventArgs e)
        {
            double width = (double)nudSetDistance.Value * mf.ftOrMtoM;

            if (((hdArr.Length - end + start) % hdArr.Length) < ((hdArr.Length - start + end) % hdArr.Length)) { int index = start; start = end; end = index; }
            if (((hdArr.Length - start + end) % hdArr.Length) < 1) return;

            if (start > end)
            {
                for (int i = start; i < hdArr.Length; i++)
                {
                    hdArr[i].easting = headLineTemplate[i].easting
                        + (-Math.Sin(glm.PIBy2 + headLineTemplate[i].heading) * width);
                    hdArr[i].northing = headLineTemplate[i].northing
                        + (-Math.Cos(glm.PIBy2 + headLineTemplate[i].heading) * width);
                    hdArr[i].heading = headLineTemplate[i].heading;
                }

                for (int i = 0; i < end; i++)
                {
                    hdArr[i].easting = headLineTemplate[i].easting
                        + (-Math.Sin(glm.PIBy2 + headLineTemplate[i].heading) * width);
                    hdArr[i].northing = headLineTemplate[i].northing
                        + (-Math.Cos(glm.PIBy2 + headLineTemplate[i].heading) * width);
                    hdArr[i].heading = headLineTemplate[i].heading;
                }
            }
            else
            {
                for (int i = start; i < end; i++)
                {
                    //calculate the point inside the boundary
                    hdArr[i].easting = headLineTemplate[i].easting
                        + (-Math.Sin(glm.PIBy2 + headLineTemplate[i].heading) * width);
                    hdArr[i].northing = headLineTemplate[i].northing
                        + (-Math.Cos(glm.PIBy2 + headLineTemplate[i].heading) * width);
                    hdArr[i].heading = headLineTemplate[i].heading;
                }
            }

            isSet = false;
            start = end = -1;
            RebuildHeadLineTemplate();
        }

        private void btnMakeFixedHeadland_Click(object sender, EventArgs e)
        {
            double width = (double)nudDistance.Value * mf.ftOrMtoM;
            for (int i = 0; i < headLineTemplate.Count; i++)
            {
                //calculate the point inside the boundary
                hdArr[i] = new vec3( headLineTemplate[i].easting + (-Math.Sin(glm.PIBy2 + headLineTemplate[i].heading) * width),
                headLineTemplate[i].northing + (-Math.Cos(glm.PIBy2 + headLineTemplate[i].heading) * width),
                headLineTemplate[i].heading);
            }

            totalHeadlandWidth += width;
            lblHeadlandWidth.Text = (totalHeadlandWidth * mf.m2FtOrM).ToString("N2");

            FixTurnLine(width, headLineTemplate, 2);

            isSet = false;
            start = end = -1;
            RebuildHeadLineTemplate();
        }

        private void cboxToolWidths_SelectedIndexChanged(object sender, EventArgs e)
        {
            BuildHeadLineTemplateFromBoundary();
            double width = (Math.Round(mf.tool.toolWidth * cboxToolWidths.SelectedIndex, 1));

            for (int i = 0; i < headLineTemplate.Count; i++)
            {
                //calculate the point inside the boundary
                hdArr[i] = new vec3(
                    headLineTemplate[i].easting + (-Math.Sin(glm.PIBy2 + headLineTemplate[i].heading) * width),
                    headLineTemplate[i].northing + (-Math.Cos(glm.PIBy2 + headLineTemplate[i].heading) * width),
                    headLineTemplate[i].heading);
            }

            lblHeadlandWidth.Text = (width * mf.m2FtOrM).ToString("N2");
            totalHeadlandWidth = width;

            FixTurnLine(width, headLineTemplate, 2);

            isSet = false;
            start = end = -1;
            RebuildHeadLineTemplate();
        }
        private void oglSelf_Paint(object sender, PaintEventArgs e)
        {
            oglSelf.MakeCurrent();

            GL.Clear(ClearBufferMask.DepthBufferBit | ClearBufferMask.ColorBufferBit);
            GL.LoadIdentity();                  // Reset The View

            CalculateMinMax();

            //back the camera up
            GL.Translate(0, 0, -maxFieldDistance);

            //translate to that spot in the world
            GL.Translate(-fieldCenterX, -fieldCenterY, 0);

            GL.Color3(1, 1, 1);

            //if (start == 99999)
            //    lblStart.Text = "--";
            //else lblStart.Text = start.ToString();

            //if (end == 99999)
            //     lblEnd.Text = "--";
            //else lblEnd.Text = end.ToString();

            //draw all the boundaries
            mf.bnd.DrawBoundaryLines();

            int ptCount = hdArr.Length;
            if (ptCount > 1)
            {
                GL.LineWidth(1);
                GL.Color3(0.20f, 0.96232f, 0.30f);
                GL.PointSize(2);
                GL.Begin(PrimitiveType.LineStrip);
                for (int h = 0; h < ptCount; h++) GL.Vertex3(hdArr[h].easting, hdArr[h].northing, 0);

                GL.Color3(0.60f, 0.9232f, 0.0f);
                GL.Vertex3(hdArr[0].easting, hdArr[0].northing, 0);
                GL.End();
            }

            GL.PointSize(8.0f);
            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.95f, 0.90f, 0.0f);
            GL.Vertex3(mf.pivotAxlePos.easting, mf.pivotAxlePos.northing, 0.0);
            GL.End();

            DrawABTouchLine();

            GL.Flush();
            oglSelf.SwapBuffers();
        }

        private void oglSelf_MouseDown(object sender, MouseEventArgs e)
        {
            if (isSet)
            {
                isSet = false;
                start = end = -1;
                return;
            }

            Point pt = oglSelf.PointToClient(Cursor.Position);

            //Convert to Origin in the center of window, 800 pixels
            fixPt.X = pt.X - 350;
            fixPt.Y = (700 - pt.Y - 350);
            //convert screen coordinates to field coordinates
            vec3 plotPt = new vec3(
                fixPt.X * maxFieldDistance / 632.0,
                fixPt.Y * maxFieldDistance / 632.0,
                0);

            plotPt.easting += fieldCenterX;
            plotPt.northing += fieldCenterY;

            pint.easting = plotPt.easting;
            pint.northing = plotPt.northing;

            double minDistA = double.MaxValue;

            int ptCount = hdArr.Length;
            int A = -1;
            if (ptCount > 0)
            {
                //find the closest 2 points to current fix
                for (int t = 0; t < ptCount; t++)
                {
                    double dist = ((pint.easting - hdArr[t].easting) * (pint.easting - hdArr[t].easting))
                                    + ((pint.northing - hdArr[t].northing) * (pint.northing - hdArr[t].northing));
                    if (dist < minDistA)
                    {
                        minDistA = dist;
                        A = t;
                    }
                }

                if (isA)
                {
                    start = A;
                    isA = false;
                }
                else
                {
                    end = A;
                    isA = true;
                    isSet = true;
                }
            }
        }

        private void DrawABTouchLine()
        {
            GL.PointSize(6);
            GL.Begin(PrimitiveType.Points);

            GL.Color3(0.990, 0.00, 0.250);
            if (start > -1) GL.Vertex3(hdArr[start].easting, hdArr[start].northing, 0);

            GL.Color3(0.990, 0.960, 0.250);
            if (end > -1) GL.Vertex3(hdArr[end].easting, hdArr[end].northing, 0);
            GL.End();

            if (start >= 0 && end >= 0)
            {
                GL.Color3(0.965, 0.250, 0.950);
                //draw the turn line oject
                GL.LineWidth(2.0f);
                GL.Begin(PrimitiveType.LineStrip);
                int ptCount = hdArr.Length;
                if (ptCount < 1) return;

                int start2 = start;
                int end2 = end;
                if (((hdArr.Length - end2 + start2) % hdArr.Length) < ((hdArr.Length - start2 + end2) % hdArr.Length)) { int index = start2; start2 = end2; end2 = index; }
                if (((hdArr.Length - start2 + end2) % hdArr.Length) < 1) return;

                if (start2 > end2)
                {
                    for (int c = start2; c < hdArr.Length; c++) GL.Vertex3(hdArr[c].easting, hdArr[c].northing, 0);
                    for (int c = 0; c <= end2; c++) GL.Vertex3(hdArr[c].easting, hdArr[c].northing, 0);
                }
                else
                    for (int c = start2; c <= end2; c++) GL.Vertex3(hdArr[c].easting, hdArr[c].northing, 0);

                GL.End();
            }
        }

        private void btnReset_Click(object sender, EventArgs e)
        {
            BuildHeadLineTemplateFromBoundary();

        }

        private void nudDistance_Click(object sender, EventArgs e)
        {
            mf.KeypadToNUD((NumericUpDown)sender, this);
            btnExit.Focus();
        }

        private void nudSetDistance_Click(object sender, EventArgs e)
        {
            mf.KeypadToNUD((NumericUpDown)sender, this);
            btnExit.Focus();
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            oglSelf.Refresh();
            if (isSet)
            {
                btnExit.Enabled = false;
                btnMakeFixedHeadland.Enabled = false;
                nudDistance.Enabled = false;

                nudSetDistance.Enabled = true;
                btnSetDistance.Enabled = true;
                //btnMoveLeft.Enabled = true;
                //btnMoveRight.Enabled = true;
                //btnMoveUp.Enabled = true;
                //btnMoveDown.Enabled = true;
                //btnDoneManualMove.Enabled = true;
                btnDeletePoints.Enabled = true;
                //btnStartUp.Enabled = true;
                //btnStartDown.Enabled = true;
                //btnEndDown.Enabled = true;
                //btnEndUp.Enabled = true;
            }
            else
            {
                nudSetDistance.Enabled = false;
                btnSetDistance.Enabled = false;
                //btnMoveLeft.Enabled = false;
                //btnMoveRight.Enabled = false;
                //btnMoveUp.Enabled = false;
                //btnMoveDown.Enabled = false;
                //btnDoneManualMove.Enabled = false;
                btnDeletePoints.Enabled = false;
                //btnStartUp.Enabled = false;
                //btnStartDown.Enabled = false;
                //btnEndDown.Enabled = false;
                //btnEndUp.Enabled = false;

                btnExit.Enabled = true;
                btnMakeFixedHeadland.Enabled = true;
                nudDistance.Enabled = true;
            }
        }

        private void btnExit_Click(object sender, EventArgs e)
        {
            mf.hd.headArr[0].hdLine?.Clear();

            //middle points
            for (int i = 1; i < hdArr.Length; i++)
            {
                hdArr[i - 1].heading = Math.Atan2(hdArr[i - 1].easting - hdArr[i].easting, hdArr[i - 1].northing - hdArr[i].northing);
                if (hdArr[i].heading < 0) hdArr[i].heading += glm.twoPI;
            }

            double delta = 0;
            for (int i = 0; i < hdArr.Length; i++)
            {
                if (i == 0)
                {
                    mf.hd.headArr[0].hdLine.Add(new vec3(hdArr[i].easting, hdArr[i].northing, hdArr[i].heading));
                    continue;
                }
                delta += (hdArr[i - 1].heading - hdArr[i].heading);

                if (Math.Abs(delta) > 0.01)
                {
                    mf.hd.headArr[0].hdLine.Add(new vec3(hdArr[i].easting, hdArr[i].northing, hdArr[i].heading));
                    delta = 0;
                }
            }



            //for (int i = 0; i < hdArr.Length; i++)
            //{
            //    mf.hd.headArr[0].hdLine.Add(new vec3(hdArr[i].easting, hdArr[i].northing, hdArr[i].heading));

            //    if (mf.bnd.bndArr[0].IsPointInsideBoundaryEar(mf.hd.headArr[0].hdLine[mf.hd.headArr[0].hdLine.Count -1]) mf.hd.headArr[0].isDrawList.Add(true);
            //    else mf.hd.headArr[0].isDrawList.Add(false);
            //}

            mf.hd.headArr[0].PreCalcHeadLines();

            mf.FileSaveHeadland();

            Close();
        }

        private void btnTurnOffHeadland_Click(object sender, EventArgs e)
        {
            mf.hd.headArr[0].hdLine?.Clear();

            mf.hd.headArr[0].calcList?.Clear();

            mf.FileSaveHeadland();

            Close();
        }

        private void btnDeletePoints_Click(object sender, EventArgs e)
        {
            if (((hdArr.Length - end + start) % hdArr.Length) < ((hdArr.Length - start + end) % hdArr.Length)) { int index = start; start = end; end = index; }
            if (((hdArr.Length - start + end) % hdArr.Length) < 1) return;

            if (start > end)
            {
                headLineTemplate.RemoveRange(start, headLineTemplate.Count - start);
                headLineTemplate.RemoveRange(0, end + 1);
            }
            else
            {
                headLineTemplate.RemoveRange(start, end - start + 1);
            }

            int cnt = headLineTemplate.Count;
            hdArr = new vec3[cnt];
            headLineTemplate.CopyTo(hdArr);

            RebuildHeadLineTemplate();
        }

        private void oglSelf_Load(object sender, EventArgs e)
        {
            oglSelf.MakeCurrent();
            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);
            GL.ClearColor(0.23122f, 0.2318f, 0.2315f, 1.0f);
        }

        private void oglSelf_Resize(object sender, EventArgs e)
        {
            oglSelf.MakeCurrent();
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();

            //58 degrees view
            Matrix4 mat = Matrix4.CreatePerspectiveFieldOfView(1.01f, 1.0f, 1.0f, 20000);
            GL.LoadMatrix(ref mat);

            GL.MatrixMode(MatrixMode.Modelview);
        }

        //determine mins maxs of patches and whole field.
        private void CalculateMinMax()
        {
            minFieldX = 9999999; minFieldY = 9999999;
            maxFieldX = -9999999; maxFieldY = -9999999;

            //draw patches j= # of sections
            for (int j = 0; j < mf.tool.numSuperSection; j++)
            {
                //every time the section turns off and on is a new patch
                int patchCount = mf.section[j].patchList.Count;

                if (patchCount > 0)
                {
                    //for every new chunk of patch
                    foreach (List<vec3> triList in mf.section[j].patchList)
                    {
                        int count2 = triList.Count;
                        for (int i = 0; i < count2; i += 3)
                        {
                            double x = triList[i].easting;
                            double y = triList[i].northing;

                            //also tally the max/min of field x and z
                            if (minFieldX > x) minFieldX = x;
                            if (maxFieldX < x) maxFieldX = x;
                            if (minFieldY > y) minFieldY = y;
                            if (maxFieldY < y) maxFieldY = y;
                        }
                    }
                }

                //min max of the boundary
                if (mf.bnd.bndArr.Count > 0)
                {
                    int bndCnt = mf.bnd.bndArr[0].bndLine.Count;
                    for (int i = 0; i < bndCnt; i++)
                    {
                        double x = mf.bnd.bndArr[0].bndLine[i].easting;
                        double y = mf.bnd.bndArr[0].bndLine[i].northing;

                        //also tally the max/min of field x and z
                        if (minFieldX > x) minFieldX = x;
                        if (maxFieldX < x) maxFieldX = x;
                        if (minFieldY > y) minFieldY = y;
                        if (maxFieldY < y) maxFieldY = y;
                    }
                }

                if (maxFieldX == -9999999 || minFieldX == 9999999 || maxFieldY == -9999999 || minFieldY == 9999999)
                {
                    maxFieldX = 0; minFieldX = 0; maxFieldY = 0; minFieldY = 0;
                }
                else
                {
                    //the largest distancew across field
                    double dist = Math.Abs(minFieldX - maxFieldX);
                    double dist2 = Math.Abs(minFieldY - maxFieldY);

                    if (dist > dist2) maxFieldDistance = dist;
                    else maxFieldDistance = dist2;

                    if (maxFieldDistance < 100) maxFieldDistance = 100;
                    if (maxFieldDistance > 19900) maxFieldDistance = 19900;
                    //lblMax.Text = ((int)maxFieldDistance).ToString();

                    fieldCenterX = (maxFieldX + minFieldX) / 2.0;
                    fieldCenterY = (maxFieldY + minFieldY) / 2.0;
                }

                //if (isMetric)
                //{
                //    lblFieldWidthEastWest.Text = Math.Abs((maxFieldX - minFieldX)).ToString("N0") + " m";
                //    lblFieldWidthNorthSouth.Text = Math.Abs((maxFieldY - minFieldY)).ToString("N0") + " m";
                //}
                //else
                //{
                //    lblFieldWidthEastWest.Text = Math.Abs((maxFieldX - minFieldX) * glm.m2ft).ToString("N0") + " ft";
                //    lblFieldWidthNorthSouth.Text = Math.Abs((maxFieldY - minFieldY) * glm.m2ft).ToString("N0") + " ft";
                //}
            }
        }
    }
}
