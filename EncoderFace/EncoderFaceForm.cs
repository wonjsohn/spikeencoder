using System;
using System.IO.Ports;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace EncoderFace
{
    public partial class EncoderFaceForm : Form
    {
        public EncoderFaceForm()
        {
            InitializeComponent();
        }

        private void EncoderFaceForm_Load(object sender, EventArgs e)
        {
        }



        private void swingbutton1_Click(object sender, EventArgs e)
        {
 

                //load and play the swing 1
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("C");
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            
        }

        private void swingbutton2_Click(object sender, EventArgs e)
        {


                //load and play the swing 2
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("D");
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            
        }

        private void swingbutton3_Click(object sender, EventArgs e)
        {


                //load and play the swing 3
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("E");
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            
        }

        private void abnormal_button1_Click(object sender, EventArgs e)
        {


                //load and play the swing 4
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("F");
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            
        }

        private void abnormal_button2_Click(object sender, EventArgs e)
        {


                //load and play the swing 5
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("H");
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            
        }

        private void button1_Click(object sender, EventArgs e)
        {


                //load and play the swing 
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("G");
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            
        }

        private void button2_Click(object sender, EventArgs e)
        {


                //load and play the swing 
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("I");
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            
        }

        private void button4_Click(object sender, EventArgs e)
        {

                //load and play the swing 
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("J");
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            
        }

        private void button3_Click(object sender, EventArgs e)
        {


                //load and play the swing 
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("K");
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            
        }

        private void IZNeuron_radioButton_CheckedChanged(object sender, EventArgs e)
        {
            //Encoding mode: IZNeuron 
            try
            {
                SerialPort sp = new SerialPort("COM35", 115200);
                sp.Open();
                sp.Write("A");//real time gyro
                sp.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }

        }

        private void fixedrate_radioButton_CheckedChanged(object sender, EventArgs e)
        {
            //encoding mode: fixed rate
            try
            {
                SerialPort sp = new SerialPort("COM35", 115200);
                sp.Open();
                sp.Write("B"); //real time fixed rate
                sp.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void live_radioButton_CheckedChanged(object sender, EventArgs e)
        {
            //runoding mode: live
            try
            {
                SerialPort sp = new SerialPort("COM35", 115200);
                sp.Open();
                sp.Write("X"); // live mode
                sp.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void playback_radioButton_CheckedChanged(object sender, EventArgs e)
        {
            //runoding mode:playback
            try
            {
                SerialPort sp = new SerialPort("COM35", 115200);
                sp.Open();
                sp.Write("Y"); // playback mode
                sp.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void fixedrate_Hz_ValueChanged(object sender, EventArgs e)
        {
            setPulseFrequency();
        }


        // set pulse frequency (Hz) 
        private void setPulseFrequency()
        {
            try
            {
                //UserFaceProgram.UF.AddLine_RecvBox("Setting pulse frequency to " + this.fixedrate_Hz.Value.ToString());
                byte[] A = new byte[2];
                UInt16 freq = Convert.ToUInt16(this.fixedrate_Hz.Value);
                A = BitConverter.GetBytes(freq);


                SerialPort sp = new SerialPort("COM35", 115200);
                sp.Open();
                sp.Write(A, 0, 2); // send number 
                sp.Close();

            }
            catch (Exception ex)
            {
                //string specificText = "Unable to set pulse freuqency.";
                //MessageBox.Show(ex.Message + "\n" + specificText);
                //this.logBox_richTextBox.AppendText(specificText + "\n");
                //logBox_richTextBox.ScrollToCaret(); // auto scroll to bottom..

            }
        }

        private void rmwireless_radioButton_CheckedChanged(object sender, EventArgs e)
        {
            //transmission mode: rf wireless
            try
            {
                SerialPort sp = new SerialPort("COM35", 115200);
                sp.Open();
                sp.Write("V"); // rf wireless mode
                sp.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void RFMOff_radioButton_CheckedChanged(object sender, EventArgs e)
        {
            //transmission mode: RFM off (not receiving from the gyro board)
            try
            {
                SerialPort sp = new SerialPort("COM35", 115200);
                sp.Open();
                sp.Write("W"); // 
                sp.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void idle_silent_radioButton_CheckedChanged(object sender, EventArgs e)
        {
            //idle definition
            try
            {
                SerialPort sp = new SerialPort("COM35", 115200);
                sp.Open();
                sp.Write("T"); //  idle: silent. Keep this silent in playback mode
                sp.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void idle_lowfreq_radioButton_CheckedChanged(object sender, EventArgs e)
        {
            //idle definition
            if (this.playback_radioButton.Checked)
            {
                MessageBox.Show("Runmode should be live", "Help", MessageBoxButtons.OK, MessageBoxIcon.Error);
            } else if (this.IZNeuron_radiobutton.Checked) {
                MessageBox.Show("Encoding mode should be Fixed rate", "Help", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            else
            {
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("U"); //  idle: low freq (EXP3 mode) @ live mode only. 
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            }
        }

        private void idle_definition_groupBox_Enter(object sender, EventArgs e)
        {

        }

        private void idle_run_both50hz_radioButton_CheckedChanged(object sender, EventArgs e)
        {
            if (this.playback_radioButton.Checked)
            {
                MessageBox.Show("Runmode should be live", "Help", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            else if (this.IZNeuron_radiobutton.Checked)
            {
                MessageBox.Show("Encoding mode should be Fixed rate", "Help", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            else
            {
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("V"); //  idle: low freq (EXP3 mode) @ live mode only. 
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            }
        }

        private void idle200_run300_radioButton_CheckedChanged(object sender, EventArgs e)
        {
            if (this.playback_radioButton.Checked)
            {
                MessageBox.Show("Runmode should be live", "Help", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            else if (this.IZNeuron_radiobutton.Checked)
            {
                MessageBox.Show("Encoding mode should be Fixed rate", "Help", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            else
            {
                try
                {
                    SerialPort sp = new SerialPort("COM35", 115200);
                    sp.Open();
                    sp.Write("W"); //
                    sp.Close();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            }
        }
    }
}
