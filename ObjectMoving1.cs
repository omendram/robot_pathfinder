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

        //private int r = 40; 
        private int _x;
        private int _y;
        private int x1;
        private int x2;
        private int y1;
        private int y2;

        private int i1;
        private int i2;
        private int j1;
        private int j2;

        private Position _objPosition;
        private Rectangle envrectouttop = new Rectangle(11, 11, 948, 238);
        private Rectangle envrectoutleft = new Rectangle(11, 11, 238, 748);
        private Rectangle envrectoutright = new Rectangle(702, 248, 258, 509);
        private Rectangle envrectoutbott = new Rectangle(238, 502, 500, 257);

        public ObjectMoving()
        {
            InitializeComponent();

            //double[] sensors = new double[12];

            _x = 100;
            _y = 100; 
            
            x1 = 10;
            y1 = 10;
            x2 = 960;
            y2 = 760;

            i1 = 250;
            j1 = 250;
            i2 = 700;
            j2 = 500; 

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
            l.DrawLine(isensor, _x + 5, _y + 20, _x - 12, _y + 10); // draw 8. infrared sensor 
            l.DrawLine(isensor, _x + 20, _y + 5, _x + 10, _y - 12); // draw 9. infrared sensor 
            l.DrawLine(isensor, _x + 60, _y + 5, _x + 70, _y - 12); // draw 11. infrared sensor 
            l.DrawLine(isensor, _x + 74, _y + 20, _x + 91, _y + 10); // draw 12. infrared sensor 

           
            
            
            // Rectangle environment 
            Line A = new Line(new Point(x1, y1), new Point(x1, y2));
            Line B = new Line(new Point(x1, y1), new Point(x2, y1));
            Line C = new Line(new Point(x2, y1), new Point(x2, y2));
            Line D = new Line(new Point(x1, y2), new Point(x2, y2));

            Line E = new Line(new Point(i1, j1), new Point(i1, j2));
            Line F = new Line(new Point(i1, j1), new Point(i2, j1));
            Line G = new Line(new Point(i2, j1), new Point(i2, j2));
            Line H = new Line(new Point(i1, j2), new Point(i2, j2));




            //l.DrawLine(pen, 20, 350, 960, 350);
            //l.DrawLine(p, _x + 60, _y + 74, _x + 540, _y + 906);

            // Sensors

            //Line sensor0 = new Line(new Point(20, 350), new Point(960, 350));
            //Line sensor = new Line(new Point(200, 10), new Point(200, 600));
            Line sensor1 = new Line(new Point(_x + 40, _y), new Point(_x + 40, _y - 960)); // 10
            Line sensor2 = new Line(new Point(_x + 40, _y + 100), new Point(_x + 40,_y + 1040)); // 4
            Line sensor3 = new Line(new Point(_x, _y + 40), new Point(_x - 960, _y + 40));// 7
            Line sensor4 = new Line(new Point(_x + 80, _y + 40), new Point(_x + 1040, _y + 40));// 1
            Line sensor5 = new Line(new Point(_x + 74, _y + 60), new Point(_x + 906, _y + 540)); // 2
            Line sensor6 = new Line(new Point(_x + 60, _y + 74), new Point(_x + 540, _y + 906));// 3
            Line sensor7 = new Line(new Point(_x + 20, _y + 74), new Point(_x - 460, _y + 906));// 5
            Line sensor8 = new Line(new Point(_x + 5, _y + 60), new Point(_x - 826, _y + 540));// 6
            Line sensor9 = new Line(new Point(_x + 5, _y + 20), new Point(_x - 826, _y - 460));// 8
            Line sensor10 = new Line(new Point(_x + 20, _y + 5), new Point(_x - 460, _y - 826));// 9
            Line sensor11 = new Line(new Point(_x + 60, _y + 5), new Point(_x + 540, _y - 826));// 11
            Line sensor12 = new Line(new Point(_x + 74, _y + 20), new Point(_x + 906, _y - 460));// 12

            Pen intersectionoutside = new Pen(Color.DarkGreen,6);
            Pen intersectioninside = new Pen(Color.DarkOrange, 6);

            Point iPoint2 = Intersect(F, sensor6);  
            Point iPoint = Intersect(sensor1, B);
            l.DrawArc(intersectionoutside, iPoint.X, iPoint.Y, 4, 4, 0, 360);
            l.DrawArc(intersectioninside, iPoint2.X, iPoint2.Y, 4, 4, 0, 360);
            Console.WriteLine("Intersection outside rectangle " + iPoint.X + "  " + iPoint.Y);
            Console.WriteLine("Intersection inside rectangle " + iPoint2.X + "  " + iPoint2.Y);



            double dx = ((_x + 40) - iPoint.X); 
            double dy = ((_y + 40) - iPoint.Y);
            double multi = dx * dx + dy * dy;
            double distance = Math.Sqrt(multi);

            double dx1 = ((_x + 40) - iPoint2.X);
            double dy1 = ((_y + 40) - iPoint2.Y);
            double multi1 = dx1 * dx1 + dy1 * dy1;
            double distance1 = Math.Sqrt(multi1);

            Console.WriteLine("Distance between outside wall and center of robot = " + distance);
            Console.WriteLine("Distance between inside wall and center of robot = " + distance1);

            bool checkdistance = true;
            if ((distance < 10) || ( distance1 < 10))
            {
                 checkdistance = false; 
            } 
            else
            {
                checkdistance = true; 
            }

            l.Dispose();
        }


        private Point Intersect(Line a, Line b)
        {
            double A1 = (a.s.Y - a.e.Y);
            double B1 = (a.s.X - a.e.X);
            double C1 = A1 * a.e.X + B1 * a.s.Y;

            double A2 = (b.s.Y - b.e.Y);
            double B2 = (b.s.X - b.e.X);
            double C2 = A2 * b.e.X + B2 * b.s.Y;

            double delta = (A1 * B2 - A2 * B1); 
            if (delta == 0)
            {
                return new Point(0, 0); 
            }
            else
            {
                double x = (B2 * C1 - B1 * C2) / delta;
                double y = (A1 * C2 - A2 * C1) / delta;
                return new Point(Convert.ToInt32(x), Convert.ToInt32(y)); 

            }

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
    public class Line
    {
        public Point s { get; set; }
        public Point e { get; set; }
        public Line (Point s, Point e)
        {
            this.s = s;
            this.e = e; 
        }
    }

}


