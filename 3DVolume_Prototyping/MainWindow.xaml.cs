using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Documents.DocumentStructures;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit;
using Microsoft.Kinect.Toolkit.Controls;
using Emgu.CV;
using Emgu.CV.Structure;

namespace _3DVolume_Prototyping
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensorChooser _sensorChooser;                                 
        private DepthImagePixel[] _depthPixels;
        private bool _scanBool,_detectObj,_processingFrame,_calculateVolume;
        private const int DepthMin = 400,DepthMax = 4000;
        private Image<Gray, short> _temp1Image, _temp2Image, _temp3Image, _obj1stImage, _obj2ndImage;
        private Point _clickPos;
        private int _averageSamplesCount;
        private readonly BackgroundWorker _worker=new BackgroundWorker();

        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// At startup find a kinect sensor and start it.Also initiliaze a background worker and run it.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MainWindow_OnLoaded(object sender, RoutedEventArgs e)
        {
            _sensorChooser = new KinectSensorChooser();
            _sensorChooser.KinectChanged += SensorChooserOnKinectChanged;
            SensorChooserUi.KinectSensorChooser = _sensorChooser;
            _sensorChooser.Start();
            _worker.WorkerSupportsCancellation = true;
            _worker.DoWork +=WorkerOnDoWork;
            _worker.RunWorkerAsync();
        }

        /// <summary>
        /// Job to be done by worker guided through the use of various flags.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="doWorkEventArgs"></param>
        private void WorkerOnDoWork(object sender, DoWorkEventArgs doWorkEventArgs)
        {
            while (!_worker.CancellationPending)
            {
                if (_processingFrame) ProcessDepthData();
                if (_detectObj && !_clickPos.X.Equals(0) && !_clickPos.Y.Equals(0))
                {
                    _processingFrame = false;
                    FindObject((int) Math.Round(_clickPos.X), (int) Math.Round(_clickPos.Y),
                        _temp3Image[(int) Math.Round(_clickPos.Y), (int) Math.Round(_clickPos.X)].Intensity);
                    _clickPos = new Point(0, 0);
                }
                if (_calculateVolume) VolumeCalc();
            }
        }

        /// <summary>
        /// While closing the window close kinect sensor and stop the background worker.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MainWindow_OnClosing(object sender, CancelEventArgs e)
        {
            _worker.CancelAsync();
            _sensorChooser.Stop();
        }

        /// <summary>
        /// Start the kinect sensor's depth stream.  
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void SensorChooserOnKinectChanged(object sender, KinectChangedEventArgs e)
        {
            if (e.OldSensor != null)
            {
                try
                {
                    e.OldSensor.DepthFrameReady -= MainWindowOnDepthFrameReady;
                    e.OldSensor.DepthStream.Range = DepthRange.Default;
                    e.OldSensor.SkeletonStream.EnableTrackingInNearRange = false;
                    e.OldSensor.DepthStream.Disable();
                    e.OldSensor.SkeletonStream.Disable();
                }
                catch (InvalidOperationException) { }
            }
            if (e.NewSensor == null) return;
            try
            {
                e.NewSensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                e.NewSensor.DepthFrameReady += MainWindowOnDepthFrameReady;
                //InitializeResources();
                ResetBtn_OnClick(new object(), new RoutedEventArgs());
                try
                {
                    e.NewSensor.DepthStream.Range = DepthRange.Near;
                }
                catch (InvalidOperationException)
                {
                    e.NewSensor.DepthStream.Range = DepthRange.Default;
                    e.NewSensor.SkeletonStream.EnableTrackingInNearRange = false;
                }
            }
            catch (InvalidOperationException) { }
        }

        /// <summary>
        /// Initialize data according to the depth stream size.
        /// </summary>
        private void InitializeResources()
        {
            _depthPixels = new DepthImagePixel[_sensorChooser.Kinect.DepthStream.FramePixelDataLength];
            _temp1Image = new Image<Gray, short>(_sensorChooser.Kinect.DepthStream.FrameWidth,
                _sensorChooser.Kinect.DepthStream.FrameHeight, new Gray(0));
            _temp2Image = new Image<Gray, short>(_sensorChooser.Kinect.DepthStream.FrameWidth,
                _sensorChooser.Kinect.DepthStream.FrameHeight, new Gray(0));
            _temp3Image = new Image<Gray, short>(_sensorChooser.Kinect.DepthStream.FrameWidth,
                _sensorChooser.Kinect.DepthStream.FrameHeight, new Gray(0));
        }

        /// <summary>
        /// Get or ignore data for each depth frame returned from kinect according to flags or null check
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MainWindowOnDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame == null || _processingFrame || _detectObj) return;
                depthFrame.CopyDepthImagePixelDataTo(_depthPixels);
                _processingFrame = true;
            }
        }

        /// <summary>
        /// Pass the depth image through a 3x3 median filter and create an average image of 100 depth frames if flag is true
        /// </summary>
        private void ProcessDepthData()
        {
            for (int i = 0; i < _depthPixels.Length; i++)
            {
                if (_depthPixels[i].Depth > DepthMax || _depthPixels[i].Depth < DepthMin)
                    _temp1Image[
                        i/_sensorChooser.Kinect.DepthStream.FrameWidth, i%_sensorChooser.Kinect.DepthStream.FrameWidth]
                        = new Gray(0);
                else
                    _temp1Image[
                        i/_sensorChooser.Kinect.DepthStream.FrameWidth, i%_sensorChooser.Kinect.DepthStream.FrameWidth]=
                        new Gray(_depthPixels[i].Depth * short.MaxValue / DepthMax);
            }
            CvInvoke.MedianBlur(_temp1Image,_temp2Image,3);
            if (_scanBool)
            {
                if (_averageSamplesCount == 0) _temp3Image = _temp2Image.Clone();
                else
                {
                    for (int x = 0; x < _temp2Image.Width; x++)
                    {
                        for (int y = 0; y < _temp2Image.Height; y++)
                        {
                            if (_temp2Image[y, x].Intensity > 0 && _temp3Image[y, x].Intensity.Equals(0))
                                _temp3Image[y, x] = new Gray(_temp2Image[y, x].Intensity);

                            else if (_temp2Image[y, x].Intensity > 0)
                                _temp3Image[y, x] =
                                    new Gray((_temp3Image[y, x].Intensity + _temp2Image[y, x].Intensity)/2);
                        }
                    }
                }
                if (_averageSamplesCount < 100) _averageSamplesCount++;
                else
                {
                    for (int x = 0; x < _temp3Image.Width; x++)
                    {
                        for (int y = 0; y < _temp3Image.Height; y++)
                        {
                            _temp3Image[y, x] = new Gray(_temp3Image[y, x].Intensity*DepthMax/short.MaxValue);
                        }
                    }
                    _scanBool = false;
                    _detectObj = true;
                    DepthImage.Dispatcher.BeginInvoke(
                        new Action(() => DepthImage.Source = BitmapSourceConvert.ToBitmapSource(_temp3Image)));
                    StatusTxt.Dispatcher.BeginInvoke(new Action(() => StatusTxt.Text = "Click In Image To Detect Object"));
                }
            }
            else
                DepthImage.Dispatcher.BeginInvoke(
                    new Action(() => DepthImage.Source = BitmapSourceConvert.ToBitmapSource(_temp2Image)));
            _processingFrame = false;
        }

        /// <summary>
        /// When pressed start scanning for the first image of the object
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void FirstScanBtn_OnClick(object sender, RoutedEventArgs e)
        {
            StatusTxt.Text = "Please Wait";
            _scanBool = true;
            FirstScanBtn.IsEnabled = false;
            ResetBtn.IsEnabled = false;
        }

        /// <summary>
        /// Get mouse pointer coordinates when clicked on depth image
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void DepthImage_OnMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            _clickPos = e.GetPosition(DepthImage);
        }

        /// <summary>
        /// Find the object using an flood-fill algorithm and create a box around the object
        /// </summary>
        /// <param name="startX"></param>
        /// <param name="startY"></param>
        /// <param name="startDepth"></param>
        private void FindObject(int startX, int startY, double startDepth)
        {
            _temp1Image = new Image<Gray, short>(_sensorChooser.Kinect.DepthStream.FrameWidth,
                _sensorChooser.Kinect.DepthStream.FrameHeight, new Gray(0));
            if (startDepth.Equals(0)) return;
            List<Point> processQueue = new List<Point> {new Point(startX, startY)};
            while (processQueue.Count>0)
            {
                if (_temp3Image[(int) processQueue[0].Y, (int) processQueue[0].X].Intensity < startDepth + 100 &&
                    _temp3Image[(int) processQueue[0].Y, (int) processQueue[0].X].Intensity > startDepth - 100 &&
                    _temp1Image[(int) processQueue[0].Y, (int) processQueue[0].X].Intensity.Equals(0))
                {
                    _temp1Image[(int) processQueue[0].Y, (int) processQueue[0].X] =
                        new Gray(_temp3Image[(int) processQueue[0].Y, (int) processQueue[0].X].Intensity);
                    if (processQueue[0].Y + 1 < _temp3Image.Height)
                        processQueue.Add(new Point(processQueue[0].X, processQueue[0].Y + 1));
                    if (processQueue[0].Y - 1 > 0)
                        processQueue.Add(new Point(processQueue[0].X, processQueue[0].Y - 1));
                    if (processQueue[0].X + 1 < _temp3Image.Width)
                        processQueue.Add(new Point(processQueue[0].X + 1, processQueue[0].Y));
                    if (processQueue[0].X - 1 > 0)
                        processQueue.Add(new Point(processQueue[0].X - 1, processQueue[0].Y));
                }
                processQueue.RemoveAt(0);
            }
            int maxX=startX, minX=startX, maxY=startY, minY=startY;
            for (int y = 0; y < _temp1Image.Height; y++)
            {
                for (int x = 0; x < _temp1Image.Width; x++)
                {
                    if (!(_temp1Image[y,x].Intensity>0))continue;
                    if (maxX < x) maxX = x;
                    if (maxY < y) maxY = y;
                    if (minX > x) minX = x;
                    if (minY > y) minY = y;
                }
            }
            ObjectBorders.Dispatcher.BeginInvoke(
                new Action(
                    () =>
                        ObjectBorders.Points =
                            new System.Windows.Media.PointCollection
                            {
                                new Point(minX, minY),
                                new Point(maxX, minY),
                                new Point(maxX, maxY),
                                new Point(minX, maxY),
                                new Point(minX, minY)
                            }));
            XMin.Dispatcher.BeginInvoke(new Action(() =>
            {
                XMin.IsEnabled = true;
                XMin.Text = minX.ToString();
            }));
            XMax.Dispatcher.BeginInvoke(new Action(() =>
            {
                XMax.IsEnabled = true;
                XMax.Text = maxX.ToString();
            }));
            YMax.Dispatcher.BeginInvoke(new Action(() =>
            {
                YMax.IsEnabled = true;
                YMax.Text = maxY.ToString();
            }));
            YMin.Dispatcher.BeginInvoke(new Action(() =>
            {
                YMin.IsEnabled = true;
                YMin.Text = minY.ToString();
            }));
            StatusTxt.Dispatcher.BeginInvoke(
                new Action(() => StatusTxt.Text = "Adjust Selection With Coordinates And Continue"));
            DepthImage.Dispatcher.BeginInvoke(
                new Action(() => DepthImage.Source = BitmapSourceConvert.ToBitmapSource(_temp1Image)));
            if (_obj1stImage == null)
            {
                SecondScanBtn.Dispatcher.BeginInvoke(new Action(() => SecondScanBtn.IsEnabled = true));
                ResetBtn.Dispatcher.BeginInvoke(new Action(() => ResetBtn.IsEnabled = true));
            }

            else
            {
                ResetBtn.Dispatcher.BeginInvoke(new Action(() => ResetBtn.IsEnabled = true));
                CalculateBtn.Dispatcher.BeginInvoke(new Action(() => CalculateBtn.IsEnabled = true));
            }
        }

        /// <summary>
        /// Refresh the highlighted area according to the new values
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CoordsTextBox_OnTextChanged(object sender, TextChangedEventArgs e)
        {
            int xmax,xmin,ymax,ymin;
            if ((ObjectBorders.Points.Count <= 0) || !int.TryParse(XMax.Text, out xmax) ||
                !int.TryParse(XMin.Text, out xmin) || !int.TryParse(YMax.Text, out ymax) ||
                !int.TryParse(YMin.Text, out ymin) || xmax >= DepthImage.Width || xmin < 0 || ymax >= DepthImage.Height ||
                ymin < 0 || ymin > ymax || xmin > xmax)
            {
                StatusTxt.Text = "Problem With Selected Coordinates.Make Sure They Are Correct";
                return;
            }
            ObjectBorders.Points[0] = new Point(double.Parse(XMin.Text), double.Parse(YMin.Text));
            ObjectBorders.Points[1] = new Point(double.Parse(XMax.Text), double.Parse(YMin.Text));
            ObjectBorders.Points[2] = new Point(double.Parse(XMax.Text), double.Parse(YMax.Text));
            ObjectBorders.Points[3] = new Point(double.Parse(XMin.Text), double.Parse(YMax.Text));
            ObjectBorders.Points[4] = new Point(double.Parse(XMin.Text), double.Parse(YMin.Text));
            StatusTxt.Text = "Adjust Selection With Coordinates And Continue";
        }

        /// <summary>
        /// When pressed scan for the second image of the object.Also trim the first image to the region of interest
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void SecondScanBtn_OnClick(object sender, RoutedEventArgs e)
        {
            StatusTxt.Text = "Please Wait";
            SecondScanBtn.IsEnabled = false;
            ResetBtn.IsEnabled = false;
            if (!TrimtoRoi(_temp1Image, ref _obj1stImage))
            {
                StatusTxt.Text = "Error in 1st Image.Reset and Start Again";
                return;
            }
            XMin.IsEnabled = YMax.IsEnabled= YMin.IsEnabled = XMax.IsEnabled= false;
            _averageSamplesCount = 0;
            ObjectBorders.Points = null;
            InitializeResources();
            _clickPos =new Point(0,0);
            _scanBool = true;
            _detectObj = false;
        }

        /// <summary>
        /// Trim each column and row around the object which does not contain any data 
        /// </summary>
        /// <param name="tempImage"></param>
        /// <param name="objImage"></param>
        /// <returns></returns>
        private bool TrimtoRoi(Image<Gray, short> tempImage,ref Image<Gray, short> objImage)
        {
            if ((ObjectBorders.Points.Count <= 0) || ObjectBorders.Points[0].X <= 0 || ObjectBorders.Points[0].Y <= 0 ||
                ObjectBorders.Points[2].X >= 640 || ObjectBorders.Points[2].Y >= 480) return false;
            objImage = new Image<Gray, short>((int) (ObjectBorders.Points[2].X - ObjectBorders.Points[0].X + 1),
                (int) (ObjectBorders.Points[2].Y - ObjectBorders.Points[0].Y + 1));
            for (int y = int.Parse(YMin.Text); y < int.Parse(YMax.Text) + 1; y++)
            {
                for (int x = int.Parse(XMin.Text); x < int.Parse(XMax.Text) + 1; x++)
                {
                    objImage[y - int.Parse(YMin.Text), x - int.Parse(XMin.Text)] = new Gray(tempImage[y, x].Intensity);
                }
            }

            while (true)
            {
                bool trimLeft=true, trimRight=true, trimTop=true, trimBottom=true;
                for (int y = 0; y < objImage.Height; y++)
                {
                    if (objImage[y, 0].Intensity > 0) trimLeft = false;
                    if (objImage[y, objImage.Width-1].Intensity > 0) trimRight = false;
                }
                for (int x = 0; x < objImage.Width; x++)
                {
                    if (objImage[0, x].Intensity > 0) trimTop = false;
                    if (objImage[objImage.Height - 1, x].Intensity > 0) trimBottom = false;
                }
                if (trimBottom || trimLeft || trimTop || trimRight)
                {
                    tempImage = new Image<Gray, short>(objImage.Width - (trimLeft ? 1 : 0) - (trimRight ? 1 : 0),
                        objImage.Height - (trimTop ? 1 : 0) - (trimBottom ? 1 : 0));
                    for (int y = trimTop ? 1 : 0; y < objImage.Height - (trimBottom ? 1 : 0); y++)
                    {
                        for (int x = trimLeft ? 1 : 0; x < objImage.Width - (trimRight ? 1 : 0); x++)
                        {
                            tempImage[y - (trimTop ? 1 : 0), x - (trimLeft ? 1 : 0)] = new Gray(objImage[y, x].Intensity);
                        }
                    }
                    objImage = tempImage.Clone();
                    continue;
                }
                DepthImage.Source = BitmapSourceConvert.ToBitmapSource(objImage);
                break;
            }

            int depthPixelsCount=0;
            for (int y = 0; y < objImage.Height; y++)
            {
                for (int x = 0; x < objImage.Width; x++)
                {
                    if (objImage[y, x].Intensity > 0) depthPixelsCount++;
                }
            }
            if (depthPixelsCount != 0) return true;
            objImage = null;
            return false;
        }

        /// <summary>
        /// When pressed start calculating the volume of the object after trimming the second image.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CalculateBtn_OnClick(object sender, RoutedEventArgs e)
        {
            StatusTxt.Text = "Please Wait";
            CalculateBtn.IsEnabled = false;
            if (!TrimtoRoi(_temp1Image, ref _obj2ndImage))
            {
                StatusTxt.Text = "Error in 2nd Image.Reset and Start Again";
                return;
            }
            DepthImage.Source = BitmapSourceConvert.ToBitmapSource(_obj2ndImage);
            if (_obj1stImage == null || _obj2ndImage == null)
            {
                StatusTxt.Text = "Null Image.Reset and Start Again";
            }
            _calculateVolume = true;
        }

        /// <summary>
        /// Calculate the Volume of the object.This is done by multiplying each column of the second image with the each row of the first image.
        /// We calculate the average depth,width and height of each voxel and then we multiply them by the number of total voxels.
        /// </summary>
        private void VolumeCalc()
        {
            _detectObj = false;
            _calculateVolume = false;
            ObjectBorders.Dispatcher.BeginInvoke(new Action(() => ObjectBorders.Points = null));
            if (_obj1stImage.Height - _obj2ndImage.Height >= 10 || _obj1stImage.Height - _obj2ndImage.Height <= -10)
            {
                StatusTxt.Dispatcher.BeginInvoke(
                    new Action(() => StatusTxt.Text = "The Images have different height.Reset and Start Again"));
                return;
            }
            while (_obj1stImage.Height != _obj2ndImage.Height)
            {
                if (_obj1stImage.Height > _obj2ndImage.Height) ReduceHeight(ref _obj1stImage);
                else ReduceHeight(ref _obj2ndImage);
            }

            double voxelWidth = _obj1stImage[_obj1stImage.Height/2, _obj1stImage.Width/2].Intensity*Math.Tan(28.5)*4/
                                (_sensorChooser.Kinect.DepthStream.FrameWidth*10);

            double voxelHeight =
                Math.Abs(_obj1stImage[_obj1stImage.Height/2, _obj1stImage.Width/2].Intensity*Math.Tan(21.5*Math.PI/180)*
                         2/(_sensorChooser.Kinect.DepthStream.FrameHeight*10));

            double voxelDepth = _obj2ndImage[_obj2ndImage.Height/2, _obj2ndImage.Width/2].Intensity*Math.Tan(28.5)*4/
                                (_sensorChooser.Kinect.DepthStream.FrameWidth*10);

            double nearestDepth = DepthMax;
            for (int y = 0; y < _obj1stImage.Height;y++)
            {
                for (int x = 0; x < _obj1stImage.Width; x++)
                {
                    if (_obj1stImage[y, x].Intensity > 0 && _obj1stImage[y, x].Intensity < nearestDepth)
                        nearestDepth = _obj1stImage[y, x].Intensity;
                }
            }

            int voxelsCount = 0;
            for (int x = _obj2ndImage.Width-1; x >= 0; x--)
            {
                /*int heightMaxPnt = -1;
                int heightMinPnt = _obj2ndImage.Height + 1;
                for (int y = 0; y < _obj2ndImage.Height; y++)
                {
                    if (_obj2ndImage[y, x].Intensity <= 0) continue;
                    if (heightMaxPnt < y) heightMaxPnt = y;
                    if (heightMinPnt > y) heightMinPnt = y;
                }

                if (heightMinPnt > _obj2ndImage.Height || heightMaxPnt < 0) continue;

                for (int y = heightMinPnt; y <= heightMaxPnt; y++)
                {
                    for (int x1 = 0; x1 < _obj1stImage.Width; x1++)
                    {
                        if (_obj1stImage[y, x1].Intensity > 0) voxelsCount++;
                    }
                }*/
                for (int y = 0; y < _obj2ndImage.Height; y++)
                {
                    if (_obj2ndImage[y, x].Intensity <= 0) continue;
                    for (int x1 = 0; x1 < _obj1stImage.Width; x1++)
                    {
                        if (_obj1stImage[y, x1].Intensity > 0) voxelsCount++;
                    }
                }
            }
            StatusTxt.Dispatcher.BeginInvoke(
                new Action(() => StatusTxt.Text = "Volume is " + voxelsCount*voxelDepth*voxelHeight*voxelWidth));
        }

        /// <summary>
        /// Reduce the height of the bigger image by one.Remove the line with the least data.
        /// </summary>
        /// <param name="objImage"></param>
        private void ReduceHeight(ref Image<Gray, short> objImage)
        {
            _temp3Image = objImage.Clone();
            int sum1 = 0, sum2 = 0;
            for (int x = 0; x < objImage.Width; x++)
            {
                if (objImage[0, x].Intensity > 0) sum1++;
                if (objImage[objImage.Height - 1, x].Intensity > 0) sum2++;
            }
            if (sum1 > sum2)
            {
                objImage = new Image<Gray, short>(_temp3Image.Width, _temp3Image.Height - 1);
                for (int y = 0; y < objImage.Height; y++)
                {
                    for (int x = 0; x < objImage.Width; x++)
                    {
                        objImage[y, x] = _temp3Image[y, x];
                    }
                }
            }
            else
            {
                objImage = new Image<Gray, short>(_temp3Image.Width, _temp3Image.Height - 1);
                for (int y = 0; y < objImage.Height; y++)
                {
                    for (int x = 0; x < objImage.Width; x++)
                    {
                        objImage[y, x] = _temp3Image[y+1, x];
                    }
                }
            }
        }

        /// <summary>
        /// When pressed reinitialize the data
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ResetBtn_OnClick(object sender, RoutedEventArgs e)
        {
            _processingFrame=_detectObj = _calculateVolume = _scanBool = false;
            InitializeResources();
            _obj1stImage = _obj2ndImage = null;
            _clickPos=new Point(0,0);
            ObjectBorders.Points = null;
            CalculateBtn.IsEnabled = false;
            SecondScanBtn.IsEnabled = false;
            _averageSamplesCount = 0;
            _processingFrame = true;
            FirstScanBtn.IsEnabled = true;
            StatusTxt.Text = "Press 1st Scan Button to start.";
        }
    }
}
