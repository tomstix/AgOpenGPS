using System;
using System.Globalization;
using System.Windows.Forms;

namespace AgOpenGPS
{
    public partial class FormABCurve : Form
    {
        //access to the main GPS form and all its variables
        private readonly FormGPS mf;
        private double aveLineHeading;
        private int originalLine = 0;

        public FormABCurve(Form _mf)
        {
            mf = _mf as FormGPS;
            InitializeComponent();

            //btnPausePlay.Text = gStr.gsPause;
            this.Text = gStr.gsABCurve;
        }

        private void FormABCurve_Load(object sender, EventArgs e)
        {
            panelPick.Top = 3;
            panelPick.Left = 3;
            panelAPlus.Top = 3;
            panelAPlus.Left = 3;
            panelName.Top = 3;
            panelName.Left = 3;

            panelEditName.Top = 3;
            panelEditName.Left = 3;

            panelEditName.Visible = false;

            panelPick.Visible = true;
            panelAPlus.Visible = false;
            panelName.Visible = false;

            this.Size = new System.Drawing.Size(470, 360);

            originalLine = mf.curve.numCurveLineSelected;
            mf.curve.isOkToAddDesPoints = false;

            UpdateLineList();

            if (lvLines.Items.Count > 0 && originalLine > 0)
            {
                lvLines.Items[originalLine - 1].EnsureVisible();
                lvLines.Items[originalLine - 1].Selected = true;
                lvLines.Select();
            }

        }

        private void UpdateLineList()
        {
            lvLines.Clear();
            ListViewItem itm;

            foreach (CCurveLines item in mf.curve.curveArr)
            {
                itm = new ListViewItem(item.Name);
                lvLines.Items.Add(itm);
            }

            // go to bottom of list - if there is a bottom
            if (lvLines.Items.Count > 0)
            {
                lvLines.Items[lvLines.Items.Count - 1].EnsureVisible();
                lvLines.Items[lvLines.Items.Count - 1].Selected = true;
                lvLines.Select();
            }
        }
        //for calculating for display the averaged new line

        private void btnNewCurve_Click(object sender, EventArgs e)
        {
            lvLines.SelectedItems.Clear();
            panelPick.Visible = false;
            panelAPlus.Visible = true;
            panelName.Visible = false;

            btnAPoint.Enabled = true;
            btnBPoint.Enabled = false;
            btnPausePlay.Enabled = false;
            mf.curve.desList?.Clear();

            this.Size = new System.Drawing.Size(270, 360);
        }

        private void btnAPoint_Click(object sender, System.EventArgs e)
        {
            //mf.curve.moveDistance = 0;
            //clear out the reference list
            lblCurveExists.Text = gStr.gsDriving;
            btnBPoint.Enabled = true;
            //mf.curve.ResetCurveLine();

            btnAPoint.Enabled = false;
            mf.curve.isOkToAddDesPoints = true;
            btnPausePlay.Enabled = true;
            btnPausePlay.Visible = true;
        }

        private void btnBPoint_Click(object sender, System.EventArgs e)
        {
            aveLineHeading = 0;
            mf.curve.isOkToAddDesPoints = false;
            panelAPlus.Visible = false;
            panelName.Visible = true;

            int cnt = mf.curve.desList.Count;
            if (cnt > 3)
            {
                //calculate average heading of line
                double x = 0, y = 0;
                foreach (vec3 pt in mf.curve.desList)
                {
                    x += Math.Cos(pt.heading);
                    y += Math.Sin(pt.heading);
                }
                x /= mf.curve.desList.Count;
                y /= mf.curve.desList.Count;
                aveLineHeading = Math.Atan2(y, x);
                if (aveLineHeading < 0) aveLineHeading += glm.twoPI;

                //build the tail extensions
                AddFirstLastPoints();
                SmoothAB(4);
                mf.curve.CalculateTurnHeadings(ref mf.curve.desList);

                panelAPlus.Visible = false;
                panelName.Visible = true;

                mf.curve.desName = "Cu " +
                    (Math.Round(glm.toDegrees(aveLineHeading), 1)).ToString(CultureInfo.InvariantCulture) +
                    "\u00B0 " + mf.FindDirection(aveLineHeading);

                textBox1.Text = mf.curve.desName;
            }
            else
            {
                mf.curve.isOkToAddDesPoints = false;
                mf.curve.desList?.Clear();

                panelPick.Visible = true;
                panelAPlus.Visible = false;

                textBox1.Enter -= textBox1_Enter;
                panelName.Visible = false;
                textBox1.Enter += textBox1_Enter;

                this.Size = new System.Drawing.Size(470, 360);

                UpdateLineList();
            }
        }
        private void btnAddTime_Click(object sender, EventArgs e)
        {
            textBox1.Text += DateTime.Now.ToString(" hh:mm:ss", CultureInfo.InvariantCulture);
            mf.curve.desName = textBox1.Text;
        }


        private void btnPausePlay_Click(object sender, EventArgs e)
        {
            if (mf.curve.isOkToAddDesPoints)
            {
                mf.curve.isOkToAddDesPoints = false;
                btnPausePlay.Image = Properties.Resources.BoundaryRecord;
                //btnPausePlay.Text = gStr.gsRecord;
                btnBPoint.Enabled = false;
            }
            else
            {
                mf.curve.isOkToAddDesPoints = true;
                btnPausePlay.Image = Properties.Resources.boundaryPause;
                //btnPausePlay.Text = gStr.gsPause;
                btnBPoint.Enabled = true;
            }
        }


        private void btnCancelMain_Click(object sender, EventArgs e)
        {

            mf.curve.isCurveValid = false;
            mf.curve.moveDistance = 0;
            mf.curve.isOkToAddDesPoints = false;
            mf.curve.isCurveSet = false;
            mf.curve.refList?.Clear();
            mf.curve.isCurveSet = false;
            mf.DisableYouTurnButtons();
            //mf.btnContourPriority.Enabled = false;
            //mf.curve.ResetCurveLine();
            mf.curve.isBtnCurveOn = false;
            mf.btnCurve.Image = Properties.Resources.CurveOff;
            if (mf.isAutoSteerBtnOn) mf.btnAutoSteer.PerformClick();
            if (mf.yt.isYouTurnBtnOn) mf.btnAutoYouTurn.PerformClick();

            mf.curve.numCurveLineSelected = 0;
            Close();
        }

        private void btnCancelCurve_Click(object sender, EventArgs e)
        {
            mf.curve.isOkToAddDesPoints = false;
            mf.curve.desList?.Clear();

            panelPick.Visible = true;
            panelAPlus.Visible = false;
            panelEditName.Visible = false;

            textBox1.Enter -= textBox1_Enter;
            panelName.Visible = false;
            textBox1.Enter += textBox1_Enter;

            this.Size = new System.Drawing.Size(470, 360);

            UpdateLineList();

        }
        private void textBox1_Enter(object sender, EventArgs e)
        {
            if (mf.isKeyboardOn)
            {
                mf.KeyboardToText((TextBox)sender, this);
                btnAdd.Focus();
            }
        }
        private void btnAdd_Click(object sender, EventArgs e)
        {
            if (mf.curve.desList.Count > 0)
            {
                if (textBox1.Text.Length == 0) textBox2.Text = "No Name " + DateTime.Now.ToString("hh:mm:ss", CultureInfo.InvariantCulture);

                mf.curve.curveArr.Add(new CCurveLines());

                //array number is 1 less since it starts at zero
                int idx = mf.curve.curveArr.Count - 1;

                mf.curve.curveArr[idx].Name = textBox1.Text.Trim();
                mf.curve.curveArr[idx].aveHeading = aveLineHeading;

                //write out the Curve Points
                foreach (vec3 item in mf.curve.desList)
                {
                    mf.curve.curveArr[idx].curvePts.Add(item);
                }

                mf.FileSaveCurveLines();
                mf.curve.desList?.Clear();
            }

            panelPick.Visible = true;
            panelAPlus.Visible = false;

            textBox1.Enter -= textBox1_Enter;
            panelName.Visible = false;
            textBox1.Enter += textBox1_Enter;

            this.Size = new System.Drawing.Size(470, 360);

            UpdateLineList();
            lvLines.Focus();
            mf.curve.desList?.Clear();
        }

        private void btnListDelete_Click(object sender, EventArgs e)
        {
            mf.curve.moveDistance = 0;

            if (lvLines.SelectedItems.Count > 0)
            {
                int num = lvLines.SelectedIndices[0];
                mf.curve.curveArr.RemoveAt(num);
                lvLines.SelectedItems[0].Remove();

                //everything changed, so make sure its right
                mf.curve.numCurveLines = mf.curve.curveArr.Count;
                if (mf.curve.numCurveLineSelected > mf.curve.numCurveLines) mf.curve.numCurveLineSelected = mf.curve.numCurveLines;

                //if there are no saved oned, empty out current curve line and turn off
                if (mf.curve.numCurveLines == 0)
                {
                    mf.curve.ResetCurveLine();
                    if (mf.isAutoSteerBtnOn) mf.btnAutoSteer.PerformClick();
                    if (mf.yt.isYouTurnBtnOn) mf.btnAutoYouTurn.PerformClick();
                }

                mf.FileSaveCurveLines();
            }

            UpdateLineList();
            lvLines.Focus();
        }

        private void btnListUse_Click(object sender, EventArgs e)
        {
            //reset to generate new reference
            mf.curve.isCurveValid = false;
            mf.curve.moveDistance = 0;

            if (lvLines.SelectedItems.Count > 0)
            {

                int idx = lvLines.SelectedIndices[0];
                mf.curve.numCurveLineSelected = idx + 1;


                mf.curve.aveLineHeading = mf.curve.curveArr[idx].aveHeading;
                mf.curve.refList?.Clear();
                for (int i = 0; i < mf.curve.curveArr[idx].curvePts.Count; i++)
                {
                    mf.curve.refList.Add(mf.curve.curveArr[idx].curvePts[i]);
                }
                mf.curve.isCurveSet = true;
                mf.yt.ResetYouTurn();

                Close();
            }
            else
            {
                mf.curve.moveDistance = 0;
                mf.curve.isOkToAddDesPoints = false;
                mf.curve.isCurveSet = false;
                mf.curve.refList?.Clear();
                mf.curve.isCurveSet = false;
                mf.DisableYouTurnButtons();
                mf.curve.isBtnCurveOn = false;
                mf.btnCurve.Image = Properties.Resources.CurveOff;
                if (mf.isAutoSteerBtnOn) mf.btnAutoSteer.PerformClick();
                if (mf.yt.isYouTurnBtnOn) mf.btnAutoYouTurn.PerformClick();

                mf.curve.numCurveLineSelected = 0;
                Close();
            }
        }

        public void SmoothAB(int smPts)
        {
            //count the reference list of original curve
            int cnt = mf.curve.desList.Count;

            //the temp array
            vec3[] arr = new vec3[cnt];

            //read the points before and after the setpoint
            for (int s = 0; s < smPts / 2; s++)
            {
                arr[s] = mf.curve.desList[s];
            }

            //average them - center weighted average
            for (int i = smPts / 2; i < cnt - (smPts / 2); i++)
            {
                double easting = 0;
                double northing = 0;
                for (int j = -smPts / 2; j < smPts / 2; j++)
                {
                    easting += mf.curve.desList[j + i].easting;
                    northing += mf.curve.desList[j + i].northing;
                }
                easting /= smPts;
                northing /= smPts;
                arr[i] = new vec3(easting, northing, mf.curve.desList[i].heading);
            }

            for (int s = cnt - (smPts / 2); s < cnt; s++)
            {
                arr[s] = mf.curve.desList[s];
            }

            //make a list to draw
            mf.curve.desList?.Clear();
            for (int i = 0; i < cnt; i++)
            {
                mf.curve.desList.Add(arr[i]);
            }
        }

        public void AddFirstLastPoints()
        {
            vec3 end = mf.curve.desList[mf.curve.desList.Count - 1];
            for (int i = 1; i < 200; i++)
            {
                mf.curve.desList.Add(new vec3(end.easting + Math.Sin(end.heading) * i, end.northing + Math.Cos(end.heading) * i, end.heading));
            }

            //and the beginning
            vec3 start = new vec3(mf.curve.desList[0]);
            for (int i = 1; i < 200; i++)
            {
                mf.curve.desList.Insert(0, new vec3(start.easting - Math.Sin(start.heading) * i, start.northing - Math.Cos(start.heading) * i, start.heading));
            }
        }

        private void btnDuplicate_Click(object sender, EventArgs e)
        {
            if (lvLines.SelectedItems.Count > 0)
            {
                int idx = lvLines.SelectedIndices[0];


                panelPick.Visible = false;
                panelName.Visible = true;
                this.Size = new System.Drawing.Size(270, 360);

                panelAPlus.Visible = false;
                panelName.Visible = true;

                textBox1.Text = mf.curve.curveArr[idx].Name + " Copy";
                mf.curve.desName = textBox1.Text;

                aveLineHeading = mf.curve.curveArr[idx].aveHeading;
                mf.curve.desList?.Clear();

                for (int i = 0; i < mf.curve.curveArr[idx].curvePts.Count; i++)
                {
                    mf.curve.desList.Add(new vec3(mf.curve.curveArr[idx].curvePts[i]));
                }
            }
        }

        private void btnEditName_Click(object sender, EventArgs e)
        {
            if (lvLines.SelectedItems.Count > 0)
            {
                int idx = lvLines.SelectedIndices[0];
                textBox2.Text = mf.curve.curveArr[idx].Name;

                panelPick.Visible = false;
                panelEditName.Visible = true;
                this.Size = new System.Drawing.Size(270, 360);
            }
        }

        private void btnAddTimeEdit_Click(object sender, EventArgs e)
        {
            textBox2.Text += DateTime.Now.ToString(" hh:mm:ss", CultureInfo.InvariantCulture);
        }

        private void btnSaveEditName_Click(object sender, EventArgs e)
        {
            if (textBox2.Text.Trim() == "") textBox2.Text = "No Name " + DateTime.Now.ToString("hh:mm:ss", CultureInfo.InvariantCulture);

            int idx = lvLines.SelectedIndices[0];
            mf.curve.curveArr[idx].Name = textBox2.Text.Trim();

            textBox2.Enter -= textBox2_Enter;
            panelEditName.Visible = false;
            textBox2.Enter += textBox2_Enter;

            panelPick.Visible = true;

            mf.FileSaveCurveLines();
            mf.curve.desList?.Clear();

            this.Size = new System.Drawing.Size(470, 360);

            UpdateLineList();
            lvLines.Focus();
        }

        private void textBox2_Enter(object sender, EventArgs e)
        {
            if (mf.isKeyboardOn)
            {
                mf.KeyboardToText((TextBox)sender, this);
                btnSaveEditName.Focus();
            }
        }

        private void btnSwapAB_Click(object sender, EventArgs e)
        {
            if (lvLines.SelectedItems.Count > 0)
            {
                int idx = lvLines.SelectedIndices[0];

                mf.curve.curveArr[idx].curvePts.Reverse();
                mf.curve.CalculateTurnHeadings(ref mf.curve.curveArr[idx].curvePts);

                mf.curve.curveArr[idx].aveHeading += Math.PI;
                mf.curve.curveArr[idx].aveHeading %= glm.twoPI;

                mf.FileSaveCurveLines();
                UpdateLineList();
                lvLines.Focus();

                _ = new FormTimedMessage(1500, "A B Swapped", "Curve is Reversed");
            }
        }
    }
}