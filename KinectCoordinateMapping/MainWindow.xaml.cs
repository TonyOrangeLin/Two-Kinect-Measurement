using Microsoft.Kinect;
using System;
using System.Collections.Generic;

using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace KinectCoordinateMapping
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor _sensor;
        private MultiSourceFrameReader _reader;
        private List<Target> TargetList = new List<Target>();
        private Target pointTarget;
        private CameraSpacePoint[] ColorInSkeleton;
        private const double NumbersOfTarget = 4;
        private int cntemp = 0;
        private const double AnticipatedUU = 60;
        private const double AnticipatedVV = 140;
        private int[] boolPixels;
        private WriteableBitmap colorBitmap;
        private AngleCalulator AngleCal = new AngleCalulator();
        private int frameskipcount = 0;
        private mode measureMode = mode.CreateMode;
        enum mode {CreateMode, TransformMode};
        //private Boolean planeMode;
        private ushort[] depthPixels;
        
        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;
        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.GetDefault();

            for (int t = 0; t <= NumbersOfTarget; t++)
            {
                Target target = new Target(t);
                TargetList.Add(target);
            }
            pointTarget = new Target(0);
            if (_sensor != null)
            {
                _sensor.Open();
                this.coordinateMapper = this._sensor.CoordinateMapper;
                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
                this.colorBitmap = new WriteableBitmap(this._sensor.ColorFrameSource.FrameDescription.Width, this._sensor.ColorFrameSource.FrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.boolPixels = new int[this._sensor.DepthFrameSource.FrameDescription.LengthInPixels];

                checkBoxShowXYZ.IsChecked = true;
            }
        }
        
        private void Window_Closed(object sender, EventArgs e)
        {
            if (_reader != null)
            {
                _reader.Dispose();
            }

            if (_sensor != null)
            {
                _sensor.Close();
            }
        }

        private void Image_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (measureMode == mode.CreateMode)
            {
                cntemp++;

                if (cntemp == 1)
                {
                    TargetList[0].Setting((int)e.GetPosition(camera).X, (int)e.GetPosition(camera).Y, AnticipatedUU, AnticipatedVV, boolPixels);
                    hintLabel.Content = "點X軸";
                }
                if (cntemp == 2)
                {
                    TargetList[1].Setting((int)e.GetPosition(camera).X, (int)e.GetPosition(camera).Y, AnticipatedUU, AnticipatedVV, boolPixels);
                    hintLabel.Content = "點Y軸";
                }
                if (cntemp == 3)
                {
                    TargetList[2].Setting((int)e.GetPosition(camera).X, (int)e.GetPosition(camera).Y, AnticipatedUU, AnticipatedVV, boolPixels);
                    hintLabel.Content = "點Z軸";
                }
                if (cntemp == 4)
                {
                    TargetList[3].Setting((int)e.GetPosition(camera).X, (int)e.GetPosition(camera).Y, AnticipatedUU, AnticipatedVV, boolPixels);
                    Vector3D origin = (Vector3D)(TargetList[0].point3D());
                    Vector3D xais = (Vector3D)(TargetList[1].point3D());
                    Vector3D yais = (Vector3D)(TargetList[2].point3D());
                    Vector3D zais = (Vector3D)(TargetList[3].point3D());
                    MatrixCalculator.Set(origin, xais, yais, zais);
                    hintLabel.Content = "建構完成";
                    label.Content = "轉換完成";
                    //checkBoxShowXYZ.IsEnabled = false;
                    measureMode = mode.TransformMode;
                }
                calibrationProgressBar.Value = cntemp;
            }
            else if (measureMode == mode.TransformMode)
            {
                pointTarget.Setting((int)e.GetPosition(camera).X, (int)e.GetPosition(camera).Y, AnticipatedUU, AnticipatedVV, boolPixels);
            }
        }

        public void ClearTargetList()
        {
            for (int i = 0; i < NumbersOfTarget; i++)
            {
                TargetList[i].Del();

            }
            cntemp = 0;
        }

        private CameraSpacePoint[] ColorToSkeleton(ushort[] input)
        {
            CameraSpacePoint[] ColorInSkel = new CameraSpacePoint[1920 * 1080];
            coordinateMapper.MapColorFrameToCameraSpace(input, ColorInSkel);

            return ColorInSkel;
        }

        private void Text(double x, double y, string text, Color color,Canvas canvasObj)
        {

            TextBlock textBlock = new TextBlock();

            textBlock.Text = text;
            textBlock.FontSize = 18;
            //textBlock.Height = 500;

            textBlock.Foreground = new SolidColorBrush(color);

            Canvas.SetLeft(textBlock, x);

            Canvas.SetTop(textBlock, y);

            canvasObj.Children.Add(textBlock);

        }

        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();

            // Color
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    camera.Source = frame.ToBitmap();
                }
            }

            // Depth
            using (var frame = reference.DepthFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    var depthDesc = frame.FrameDescription;
                    if (depthPixels == null)
                    {
                        uint depthSize = depthDesc.LengthInPixels;
                        depthPixels = new ushort[depthSize];
                    }

                    frame.CopyFrameDataToArray(depthPixels);
                    //if (frameskipcount > 9)
                    //{

                        ColorInSkeleton = ColorToSkeleton(depthPixels);  //ColorPoint to SkeletonPoint  必須每個frame都作!!
                        for(int i = 0; i < NumbersOfTarget; i++)
                        {
                            TargetList[i].RefreshTarget(ColorInSkeleton);
                        }
                        frameskipcount = 0;
                        pointTarget.RefreshTarget(ColorInSkeleton);
                    //}
                    
                    //frameskipcount++;
                }
            }
            Paint();
        }

        private void Paint()
        {
            canvas.Children.Clear();

            if (measureMode == mode.CreateMode)
            {
                Ellipse ellipse2;
                if (TargetList[0].IsTracked())
                {
                    ellipse2 = new Ellipse
                    {
                        Fill = Brushes.Red,
                        Width = 10,
                        Height = 10
                    };
                    Canvas.SetLeft(ellipse2, TargetList[0].point2D().X - ellipse2.Width / 2);
                    Canvas.SetTop(ellipse2, TargetList[0].point2D().Y - ellipse2.Height / 2);
                            
                    if (checkBoxShowXYZ.IsChecked == true)
                    {
                        Text(TargetList[0].point2D().X - 10, TargetList[0].point2D().Y + 15, TargetList[0].point3D().X.ToString("f2") + ", " + TargetList[0].point3D().Y.ToString("f2") + ", " + TargetList[0].point3D().Z.ToString("f2"), Color.FromArgb(255, 68, 192, 68), canvas);
                    }
                    canvas.Children.Add(ellipse2);
                }
                if (TargetList[1].IsTracked())
                {
                    ellipse2 = new Ellipse
                    {
                        Fill = Brushes.Red,
                        Width = 10,
                        Height = 10
                    };
                    Canvas.SetLeft(ellipse2, TargetList[1].point2D().X - ellipse2.Width / 2);
                    Canvas.SetTop(ellipse2, TargetList[1].point2D().Y - ellipse2.Height / 2);
                    if (checkBoxShowXYZ.IsChecked == true)
                    {
                        Text(TargetList[1].point2D().X - 10, TargetList[1].point2D().Y + 15, TargetList[1].point3D().X.ToString("f2") + ", " + TargetList[1].point3D().Y.ToString("f2") + ", " + TargetList[1].point3D().Z.ToString("f2"), Color.FromArgb(255, 68, 192, 68), canvas);
                    }
                    canvas.Children.Add(ellipse2);
                }
                if (TargetList[2].IsTracked())
                {
                    ellipse2 = new Ellipse
                    {
                        Fill = Brushes.Red,
                        Width = 10,
                        Height = 10
                    };
                    Canvas.SetLeft(ellipse2, TargetList[2].point2D().X - ellipse2.Width / 2);
                    Canvas.SetTop(ellipse2, TargetList[2].point2D().Y - ellipse2.Height / 2);
                    if (checkBoxShowXYZ.IsChecked == true)
                    {
                        Text(TargetList[2].point2D().X - 10, TargetList[2].point2D().Y + 15, TargetList[2].point3D().X.ToString("f2") + ", " + TargetList[2].point3D().Y.ToString("f2") + ", " + TargetList[2].point3D().Z.ToString("f2"), Color.FromArgb(255, 68, 192, 68), canvas);
                    }
                    canvas.Children.Add(ellipse2);
                }
                if (TargetList[3].IsTracked())
                {
                    ellipse2 = new Ellipse
                    {
                        Fill = Brushes.Red,
                        Width = 10,
                        Height = 10
                    };
                    Canvas.SetLeft(ellipse2, TargetList[3].point2D().X - ellipse2.Width / 2);
                    Canvas.SetTop(ellipse2, TargetList[3].point2D().Y - ellipse2.Height / 2);
                    if (checkBoxShowXYZ.IsChecked == true)
                    {
                        Text(TargetList[3].point2D().X - 10, TargetList[3].point2D().Y + 15, TargetList[3].point3D().X.ToString("f2") + ", " + TargetList[3].point3D().Y.ToString("f2") + ", " + TargetList[3].point3D().Z.ToString("f2"), Color.FromArgb(255, 68, 192, 68), canvas);
                    }
                    canvas.Children.Add(ellipse2);
                }

            }
            if (measureMode == mode.TransformMode)
            {
                Ellipse ellipse2;
                if (pointTarget.IsTracked())
                {
                    ellipse2 = new Ellipse
                    {
                        Fill = Brushes.Red,
                        Width = 10,
                        Height = 10
                    };
                    Canvas.SetLeft(ellipse2, pointTarget.point2D().X - ellipse2.Width / 2);
                    Canvas.SetTop(ellipse2, pointTarget.point2D().Y - ellipse2.Height / 2);
                    Vector3D resultMatrx = MatrixCalculator.CalcNp((Vector3D)pointTarget.point3D());
                    if (checkTransformBoxShowXYZ.IsChecked == true)
                    {     
                        Text(pointTarget.point2D().X - 10, pointTarget.point2D().Y + 15, resultMatrx.X.ToString("f2") + ", " + resultMatrx.Y.ToString("f2") + ", " + resultMatrx.Z.ToString("f2"), Color.FromArgb(255, 68, 192, 68), canvas);
                    }
                    if (checkBoxShowXYZ.IsChecked == true)
                    {
                        Text(pointTarget.point2D().X - 10, pointTarget.point2D().Y + 15, pointTarget.point3D().X.ToString("f2") + ", " + pointTarget.point3D().Y.ToString("f2") + ", " + pointTarget.point3D().Z.ToString("f2"), Color.FromArgb(255, 68, 192, 68), canvas);
                    }
                    canvas.Children.Add(ellipse2);
                }


            }
        }

        private void Btn_Start_Click(object sender, RoutedEventArgs e)
        {
            cntemp = 0;
            ClearTargetList();
        }

        private void floorbutton_Click(object sender, RoutedEventArgs e)
        {
            ClearTargetList();
            measureMode = mode.CreateMode;
            label.Content = "設定座標模式";
            hintLabel.Content = "點原點";
            floorbutton.Content = "停止校正";
            calibrationProgressBar.Maximum = 4;
            calibrationProgressBar.Minimum = 0;
            calibrationProgressBar.Value = 0;
        }

        private void checkTransformBoxShowXYZ_Checked(object sender, RoutedEventArgs e)
        {
            checkBoxShowXYZ.IsChecked = false;
        }

        private void checkBoxShowXYZ_Checked(object sender, RoutedEventArgs e)
        {
            checkTransformBoxShowXYZ.IsChecked = false;
        }
    }

}
