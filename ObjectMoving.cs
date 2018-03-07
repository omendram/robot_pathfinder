using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms; 


namespace ObjectMoving
{
    public partial class ObjectMoving : Form
    {
        enum Position
        {
          Left, Right, Up, Down 
        }

        private int _x;
        private int _y;
        private Position _objPosition;
        private Rectangle envrectouttop = new Rectangle(11, 11, 948, 238);
        private Rectangle envrectoutleft = new Rectangle(11, 11, 238, 748);
        private Rectangle envrectoutright = new Rectangle(750, 248, 209, 509);
        private Rectangle envrectoutbott = new Rectangle(238, 502, 500, 257);

        public ObjectMoving()
        {
            InitializeComponent();
            
            _x = 100;
            _y = 100;
            _objPosition = Position.Right;
        }

        private void FormView_Paint(object sender, PaintEventArgs e)
        {

            Graphics l = e.Graphics;
            Pen p = new Pen(Color.Black, 4); // Indicates its robot direction 
            Pen env = new Pen(Color.Black, 2); // Environment painting 
            Pen isensor = new Pen(Color.Black, 1); // Indicates infrared sensor

            l.FillEllipse(Brushes.Pink, _x, _y, 80, 80); // Robot
            l.DrawLine(p, (_x+40), (_y+40), (_x+80), (_y+40)); // Forward direction of robot 

            l.DrawRectangle(env, 10, 10, 950, 750); // Environment outside
            l.DrawRectangle(env, 250, 250, 450, 250); // Environment inside 

            l.DrawLine(isensor, _x + 40, _y, _x + 40, _y - 20);// draw 10. infrared sensor
            l.DrawLine(isensor, _x + 40, _y + 100, _x + 40, _y + 80); // draw 4. infrared sensor 
            l.DrawLine(isensor, _x, _y + 40, _x - 20, _y + 40); // draw 7. infrared sensor 
            l.DrawLine(isensor, _x + 80, _y + 40, _x + 100, _y + 40); // draw 1. infrared sensor 
            l.DrawLine(isensor, _x + 74, _y + 60, _x + 91, _y + 70); // draw 2. infrared sensor 
            l.DrawLine(isensor, _x + 60, _y + 74, _x + 70, _y + 91); // draw 3. infrared sensor 
            l.DrawLine(isensor, _x + 20, _y + 74, _x + 10, _y + 91); // draw 5. infrared sensor 
            l.DrawLine(isensor, _x + 5, _y + 60, _x - 12, _y + 70); // draw 6. infrared sensor 
            l.DrawLine(isensor, _x + 5, _y + 20, _x - 12, _y +10); // draw 8. infrared sensor 
            l.DrawLine(isensor, _x + 20, _y + 5, _x + 10, _y - 12); // draw 9. infrared sensor 
            l.DrawLine(isensor, _x + 60, _y + 5, _x + 70, _y - 12); // draw 11. infrared sensor 
            l.DrawLine(isensor, _x + 74, _y + 20, _x + 91, _y + 10); // draw 12. infrared sensor 

            l.Dispose();
        }

        private void TmrMoving_Tick(object sender, EventArgs e)
        {
            //_y += 10;

            if (_objPosition == Position.Right)
            {
                _x += 5; 
            }
            
            Invalidate(envrectouttop);
            Invalidate(envrectoutleft);
            Invalidate(envrectoutright);
            Invalidate(envrectoutbott);
        }
    }
}
