using AForge.Imaging.Filters;
using Intel.RealSense;
using System;
using System.Collections.Generic;
using System.Data;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using Image = System.Windows.Controls.Image;
using PixelFormat = System.Drawing.Imaging.PixelFormat;
using Point = System.Drawing.Point;

namespace touchableWall
{
    public partial class MainWindow : Window
    {

        #region Variables

        Pipeline pipeline;
        PipelineProfile pipelineProfile;
        Colorizer colorizer;
        Device selectedDevice;
        CustomProcessingBlock block;
        CancellationTokenSource ctokenSource = new CancellationTokenSource();
        DecimationFilter decimationFilter;
        ThresholdFilter thresholdFilter;
        HoleFillingFilter holeFilter;
        SpatialFilter spatialFilter;
        TemporalFilter temporalFilter;
        Align align_to;

        Action<VideoFrame> updateDepth, updateColor, updateIR1, updateIR2;
        ROI roiFrame;
        Point3D objectLocation;

        public const int _limit = 80;

        int resolutionW = 480, resolutionH = 270, FPS = 60;
        int screenWidth, screenHeight;
        int calibrationCounter = 0;
        bool wallDistanceIsCalibrated = false;
        bool isCalibrated = false, isCalibrating = false, isReadXml = false;

        double[] depthFrameSum = new double[270];
        ushort[] depthFrameAvarage = new ushort[270];
        ushort[] depthFrameResult = new ushort[270];

        #endregion

        #region Helpers

        public static Bitmap Grayscale(Bitmap image)
        {
            Grayscale filter = new Grayscale(0.2125, 0.7154, 0.0721);
            return filter.Apply(image);
        }
        public static Bitmap Threshold(Bitmap image, int limit)
        {
            Threshold th;
            th = new Threshold(limit);
            th.ApplyInPlace(image);
            return image;
        }

        public static Bitmap ToBitmap(VideoFrame frame)
        {
            var bytes = new byte[frame.Stride * frame.Height];
            frame.CopyTo<byte>(bytes);

            var bpp = frame.BitsPerPixel;
            PixelFormat pf = PixelFormat.Format24bppRgb;
            switch (bpp)
            {
                case 16:
                    pf = PixelFormat.Format16bppGrayScale;
                    break;
                case 24:
                    pf = PixelFormat.Format24bppRgb;
                    break;
                case 32:
                    pf = PixelFormat.Format32bppArgb;
                    break;
            }

            var bs = new Bitmap(frame.Width, frame.Height, pf);
            var BoundsRect = new System.Drawing.Rectangle(0, 0, frame.Width, frame.Height);
            BitmapData bmpData = bs.LockBits(BoundsRect, ImageLockMode.WriteOnly, pf);

            System.Runtime.InteropServices.Marshal.Copy(bytes, 0, bmpData.Scan0, frame.Stride * frame.Height);
            bs.UnlockBits(bmpData);

            return bs;

        }

        private Point objectLocationConverter(Point3D objectLocation)
        {
            Point tempClickLoc = new Point();

            tempClickLoc.X = (((int)objectLocation.X - roiFrame.minX) * screenWidth) / (roiFrame.maxX - roiFrame.minX);
            tempClickLoc.Y = (((int)objectLocation.Y - roiFrame.minY) * screenHeight) / (roiFrame.maxY - roiFrame.minY);

            return tempClickLoc;

        }

        #endregion

        #region XMLData

        private void xmlUpdate()
        {
            try
            {
                DataTable dtData = new DataTable("dtData");

                for (int i = 0; i < depthFrameResult.Length; i++)
                {
                    dtData.Columns.Add("index" + i.ToString());
                }
                dtData.Columns.Add("wallDistanceIsCalibrated");
                dtData.Columns.Add("ROI_minX");
                dtData.Columns.Add("ROI_minY");
                dtData.Columns.Add("ROI_maxX");
                dtData.Columns.Add("ROI_maxY");
                dtData.Columns.Add("wallSensitive");
                dtData.Columns.Add("wallPerception");
                dtData.Columns.Add("objectSize");
                dtData.Columns.Add("isClickable");


                if (!File.Exists("dtData.xsd"))
                    dtData.WriteXmlSchema("dtData.xsd");

                DataRow dr = dtData.NewRow();

                for (int i = 0; i < depthFrameResult.Length; i++)
                {
                    dr["index" + i.ToString()] = Convert.ToUInt16(depthFrameResult[i]).ToString();
                }
                dr["wallDistanceIsCalibrated"] = wallDistanceIsCalibrated == true ? "1" : "0";
                dr["ROI_minX"] = roiFrame.minX.ToString();
                dr["ROI_minY"] = roiFrame.minY.ToString();
                dr["ROI_maxX"] = roiFrame.maxX.ToString();
                dr["ROI_maxY"] = roiFrame.maxY.ToString();

                dr["wallSensitive"] = Convert.ToInt32(sldWallSense.Value).ToString();
                dr["wallPerception"] = Convert.ToInt32(sldWallPerception.Value).ToString();
                dr["objectSize"] = Convert.ToInt32(sldBallSize.Value).ToString();
                dr["isClickable"] = togIsClickable.IsChecked == true ? "1" : "0";

                sldRoiMinX.Maximum = resolutionW;
                sldRoiMinY.Maximum = resolutionH;
                sldRoiMaxX.Maximum = resolutionW;
                sldRoiMaxY.Maximum = resolutionH;

                dtData.Rows.Add(dr);
                dtData.WriteXml("dtData.xml");

                if (!isReadXml)
                    xmlRead();

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void xmlRead()
        {
            try
            {
                if (File.Exists("dtData.xsd"))
                {
                    DataTable dtData = new DataTable();
                    dtData.ReadXmlSchema("dtData.xsd");
                    dtData.ReadXml("dtData.xml");

                    foreach (DataRow dr in dtData.Rows)
                    {
                        wallDistanceIsCalibrated = dr["wallDistanceIsCalibrated"].ToString() == "1" ? true : false;
                        for (int i = 0; i < depthFrameResult.Length; i++)
                        {
                            depthFrameResult[i] = Convert.ToUInt16(dr["index" + i.ToString()].ToString());
                        }
                        sldWallSense.Value = Convert.ToDouble(dr["wallSensitive"]);
                        sldWallPerception.Value = Convert.ToDouble(dr["wallPerception"]);
                        sldBallSize.Value = Convert.ToDouble(dr["objectSize"]);
                        togIsClickable.IsChecked = dr["isClickable"].ToString() == "1" ? true : false;

                        roiFrame.minX = Convert.ToInt32(dr["ROI_minX"]);
                        roiFrame.minY = Convert.ToInt32(dr["ROI_minY"]);
                        roiFrame.maxX = Convert.ToInt32(dr["ROI_maxX"]);
                        roiFrame.maxY = Convert.ToInt32(dr["ROI_maxY"]);

                        txtRoiMinX.Text = roiFrame.minX.ToString();
                        txtRoiMinY.Text = roiFrame.minY.ToString();
                        txtRoiMaxX.Text = roiFrame.maxX.ToString();
                        txtRoiMaxY.Text = roiFrame.maxY.ToString();

                        sldRoiMinX.Maximum = resolutionW;
                        sldRoiMinY.Maximum = resolutionH;
                        sldRoiMaxX.Maximum = resolutionW;
                        sldRoiMaxY.Maximum = resolutionH;

                        sldRoiMinX.Value = Convert.ToDouble(roiFrame.minX);
                        sldRoiMinY.Value = Convert.ToDouble(roiFrame.minY);
                        sldRoiMaxX.Value = Convert.ToDouble(roiFrame.maxX);
                        sldRoiMaxY.Value = Convert.ToDouble(roiFrame.maxY);

                    }
                    isReadXml = true;
                }
                else
                    xmlUpdate();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        #endregion

        #region Events

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            MessageBoxResult result = MessageBox.Show("Exit from the application?", "Exit", MessageBoxButton.YesNoCancel, MessageBoxImage.Question);
            if (result == MessageBoxResult.Yes)
            {
                ctokenSource.Cancel();
                Application.Current.Shutdown();
            }
        }

        private void btnEditRoi_Click(object sender, RoutedEventArgs e)
        {
            try
            {

                roiFrame.minX = Convert.ToInt32(sldRoiMinX.Value);
                roiFrame.minY = Convert.ToInt32(sldRoiMinY.Value);
                roiFrame.maxX = Convert.ToInt32(sldRoiMaxX.Value);
                roiFrame.maxY = Convert.ToInt32(sldRoiMaxY.Value);


                txtRoiMinX.Text = roiFrame.minX.ToString();
                txtRoiMinY.Text = roiFrame.minY.ToString();
                txtRoiMaxX.Text = roiFrame.maxX.ToString();
                txtRoiMaxY.Text = roiFrame.maxY.ToString();

                txtCalibrationStatus.Text = "ROI updated.";

                xmlUpdate();

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void btnCalibrationRoi_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                calibrationCounter = 0;
                wallDistanceIsCalibrated = false;
                depthFrameSum = new double[resolutionH];

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void Window_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            try
            {
                Dispatcher.Invoke(new Action(() =>
                {
                    if (togIsClickable.IsChecked == true)
                        togIsClickable.IsChecked = false;
                }));

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void sldWallPerception_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            try
            {
                xmlUpdate();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void sldWallSense_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            try
            {
                xmlUpdate();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void sldBallSize_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            try
            {
                xmlUpdate();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        #endregion

        #region Calibration

        private void calibrationProcess(DepthFrame depthFrame, VideoFrame colorFrame, VideoFrame infraredFrame)
        {
            try
            {
                Bitmap img = ToBitmap(colorFrame);
                img = Grayscale(img);
                img = Threshold(img, _limit);

                Dispatcher.Invoke(new Action(() =>
                {
                    imgIRleft.Source = ConvertBitmap.BitmapToBitmapSource(img);


                    if (!wallDistanceIsCalibrated)
                    {
                        isCalibrating = true;
                        wallDistanceCalibration(depthFrame);
                        isCalibrating = false;
                    }

                    if (wallDistanceIsCalibrated)
                        isCalibrated = true;

                    if (isCalibrated)
                        txtCalibrationStatus.Text = "Calibration : OK.";
                    else
                        txtCalibrationStatus.Text = "Calibration : NOT OK.";

                    img.Dispose();
                    img = null;

                }));


            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void togIsClickable_Checked(object sender, RoutedEventArgs e)
        {
            try
            {
                xmlUpdate();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
            
        }

        private void wallDistanceCalibration(DepthFrame depthFrame)
        {
            try
            {
                ushort[] depthCurrent = new ushort[depthFrame.Width * depthFrame.Height];  //Assign each index of the depth frame to the matrix
                depthFrame.CopyTo(depthCurrent);

                if (calibrationCounter < 100)  //the first 100 frames are used for calibration
                {
                    calibrationCounter++;

                    Dispatcher.Invoke(new Action(() =>
                    {
                        txtCalibrationStatus.Text = "Calibration : Calibrating.. %" + calibrationCounter.ToString();
                    }));

                    int x, y;

                    for (int i = 0; i < depthCurrent.Length; i++) //find the x and y values ​​while returning for each index and add to the sum if it is in roi

                    {
                        x = i % depthFrame.Width;
                        y = Convert.ToInt32(Math.Floor(Convert.ToDouble(i / depthFrame.Width)));

                        if (x < roiFrame.minX || x > roiFrame.maxX || y < roiFrame.minY || y > roiFrame.maxY)
                        {
                            depthCurrent[i] = 0;
                        }
                        else
                        {
                            depthFrameSum[y] += depthCurrent[i];
                        }
                    }


                    if (calibrationCounter == 99)
                    {
                        for (int i = 0; i < depthFrameSum.Length; i++)
                        {
                            depthFrameAvarage[i] = (ushort)(depthFrameSum[i] / ((roiFrame.maxX - roiFrame.minX) * calibrationCounter));
                        }

                        Dispatcher.Invoke(new Action(() =>
                        {
                            txtCalibrationStatus.Text = "Calibration : Calibration Completed.";
                        }));
                        wallDistanceIsCalibrated = true;

                        depthFrameResult = depthFrameAvarage; //assign values ​​to the result array


                        xmlUpdate();
                    }

                }


            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }
        private DepthFrame calcWallSensitive(DepthFrame depthFrame)
        {
            try
            {

                /*check each index of the received depth frame and 
                pull the image forward to the given wall sensitivity. 
                In this way, both noise will be removed.*/

                ushort[] depthCurrent = new ushort[depthFrame.Width * depthFrame.Height];
                depthFrame.CopyTo(depthCurrent);

                int x, y;

                for (int i = 0; i < depthCurrent.Length; i++)
                {
                    x = i % depthFrame.Width;
                    y = Convert.ToInt32(Math.Floor(Convert.ToDouble(i / depthFrame.Width)));

                    if (depthCurrent[i] < 100 || depthCurrent[i] > (depthFrameResult[y] - (int)sldWallSense.Value))
                    {
                        depthCurrent[i] = 0;
                    }

                    if (x < roiFrame.minX || x > roiFrame.maxX || y < roiFrame.minY || y > roiFrame.maxY)
                    {
                        depthCurrent[i] = 0;
                    }
                }
                depthFrame.CopyFrom(depthCurrent);

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
            return depthFrame;
        }

        #endregion

        #region ObjectTracking

        private Point3D findObjectLocation(DepthFrame depthFrame)
        {
            try
            {
                ushort[] depthCurrent = new ushort[depthFrame.Width * depthFrame.Height];
                depthFrame.CopyTo(depthCurrent);

                Point3D tempLocation = new Point3D();

                ushort objectSize = Convert.ToUInt16((int)sldBallSize.Value);

                if (isCalibrated)
                {
                    double sumDistance = 0;
                    bool isCatched = false;

                    for (int i = 0; i < depthCurrent.Length - objectSize; i++)
                    {
                        isCatched = false;
                        sumDistance = 0;

                        for (int j = 0; j < objectSize; j++)
                        {
                            sumDistance += depthCurrent[i + j];
                            if (depthCurrent[i + j] == 0)
                            {
                                isCatched = true;
                                break;
                            }
                        }

                        if ((!isCatched) && (depthFrameResult[(int)Math.Floor((decimal)(i / (depthFrame.Width)))] - (sldWallSense.Value + sldWallPerception.Value)) < (double)(sumDistance / objectSize))
                        {
                            tempLocation.Y = (double)(Math.Floor((decimal)(i / (depthFrame.Width))));
                            tempLocation.X = i % (depthFrame.Width);
                            tempLocation.Z = (double)(sumDistance / objectSize);

                            return tempLocation;
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
            return objectLocation;

        }

        #endregion

        #region Process
        void Init()
        {
            try
            {

                #region FILTERS

                spatialFilter = new SpatialFilter();
                spatialFilter.Options[Option.FilterMagnitude].Value = 5.0F;
                spatialFilter.Options[Option.FilterSmoothAlpha].Value = 0.25F;
                spatialFilter.Options[Option.FilterSmoothDelta].Value = 50.0F;

                decimationFilter = new DecimationFilter();
                decimationFilter.Options[Option.FilterMagnitude].Value = 2.0F;

                holeFilter = new HoleFillingFilter();

                thresholdFilter = new ThresholdFilter();
                //thresholdFilter.Options[Option.MinDistance].Value = 0.73F;
                //thresholdFilter.Options[Option.MaxDistance].Value = 0.81F;

                #endregion

                align_to = new Align(Intel.RealSense.Stream.Depth);
                colorizer = new Colorizer();
                pipeline = new Pipeline();

                //CONFIG SETTINGS
                var cfg = new Config();
                cfg.EnableStream(Intel.RealSense.Stream.Depth, resolutionW, resolutionH, Format.Z16, FPS); //depth resolution manuel change
                cfg.EnableStream(Intel.RealSense.Stream.Color, 640, 480, Format.Rgb8, 30);
                pipelineProfile = pipeline.Start(cfg); //stream starting with user config

                var advancedDevice = AdvancedDevice.FromDevice(pipelineProfile.Device); //connected device
                //read device's configuration settings from json file
                advancedDevice.JsonConfiguration = File.ReadAllText(@"CustomConfig.json");
                selectedDevice = pipelineProfile.Device;

                #region Field Of View Info

                float[] dfov, cfov, irfov;

                var depth_stream = pipelineProfile.GetStream<VideoStreamProfile>(Intel.RealSense.Stream.Depth);
                Intrinsics depthIntr = depth_stream.GetIntrinsics();
                dfov = depthIntr.FOV; // float[2] - horizontal and vertical field of view in degrees

                var color_stream = pipelineProfile.GetStream<VideoStreamProfile>(Intel.RealSense.Stream.Color);
                Intrinsics colorIntr = color_stream.GetIntrinsics();
                cfov = colorIntr.FOV; // float[2] - horizontal and vertical field of view in degrees

                var ir_stream = pipelineProfile.GetStream<VideoStreamProfile>(Intel.RealSense.Stream.Infrared);
                Intrinsics irIntr = ir_stream.GetIntrinsics();
                irfov = irIntr.FOV; // float[2] - horizontal and vertical field of view in degrees

                lblDepthFov.Text = "Depth FOV : " + "H = " + Convert.ToInt32(dfov[0]).ToString() + "° , " + "V = " + Convert.ToInt32(dfov[1]).ToString() + "°";
                lblColorFov.Text = "RGB FOV   : " + "H = " + Convert.ToInt32(cfov[0]).ToString() + "° , " + "V = " + Convert.ToInt32(cfov[1]).ToString() + "°";
                lblInfraredFov.Text = "IR FOV   : " + "H = " + Convert.ToInt32(irfov[0]).ToString() + "° , " + "V = " + Convert.ToInt32(irfov[1]).ToString() + "°";


                #endregion


                //get primary screen resolutions
                screenWidth = Convert.ToInt32(System.Windows.SystemParameters.PrimaryScreenWidth.ToString());
                screenHeight = Convert.ToInt32(System.Windows.SystemParameters.PrimaryScreenHeight.ToString());

                //camera started working. transfer image to interface
                SetupWindow(pipelineProfile, out updateDepth, out updateColor, out updateIR1, out updateIR2);

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);

            }
        }
        void RunProcess()
        {
            try
            {
                xmlRead();
                Init();

                //thread starting 
                Task.Factory.StartNew(() =>
                {

                    while (!ctokenSource.Token.IsCancellationRequested)
                    {
                        using (var releaser = new FramesReleaser())
                        {
                            using (var frames = pipeline.WaitForFrames().DisposeWith(releaser))
                            {
                                //depth frame apply filter

                                var processedFrames = frames
                                //.ApplyFilter(align_to).DisposeWith(releaser)
                                .ApplyFilter(decimationFilter).DisposeWith(releaser)
                                .ApplyFilter(spatialFilter).DisposeWith(releaser)
                                .ApplyFilter(temporalFilter).DisposeWith(releaser)
                                .ApplyFilter(holeFilter).DisposeWith(releaser)
                                //.ApplyFilter(thresholdFilter).DisposeWith(releaser)
                                .ApplyFilter(colorizer).DisposeWith(releaser);

                                using (var filteredFrames = FrameSet.FromFrame(processedFrames))
                                {
                                    var colorFrame = filteredFrames.ColorFrame.DisposeWith(filteredFrames);
                                    var depthFrame = filteredFrames.DepthFrame.DisposeWith(filteredFrames);
                                    var infraredFrame = filteredFrames.InfraredFrame.DisposeWith(filteredFrames);


                                    calibrationProcess(depthFrame, colorFrame, infraredFrame);

                                    if (isCalibrated)
                                    {
                                        depthFrame = calcWallSensitive(depthFrame);
                                        objectLocation = findObjectLocation(depthFrame);


                                        if (objectLocation.X != 0 || objectLocation.Y != 0 || objectLocation.Z != 0)
                                        {
                                            Point clickLoc = objectLocationConverter(objectLocation); //calc to object location and rate screen resolution

                                            if (togIsClickable.IsChecked == true)
                                            {
                                                MouseEvents.MouseLeftClick(clickLoc.X, clickLoc.Y);  //mouse left clicking
                                            }
                                        }
                                    }

                                    var depthFrameColorized = colorizer.Process<VideoFrame>(depthFrame).DisposeWith(filteredFrames);
                                    Dispatcher.Invoke(DispatcherPriority.Render, updateDepth, depthFrameColorized);
                                    Dispatcher.Invoke(DispatcherPriority.Render, updateColor, colorFrame);
                                    Dispatcher.Invoke(DispatcherPriority.Render, updateIR1, infraredFrame);
                                    Dispatcher.Invoke(DispatcherPriority.Render, updateIR2, infraredFrame);

                                    colorFrame.Dispose();
                                    depthFrame.Dispose();
                                    infraredFrame.Dispose();
                                    depthFrameColorized.Dispose();


                                }

                            }
                        }
                    }

                }, ctokenSource.Token);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        static Action<VideoFrame> UpdateImage(Image img)
        {
            WriteableBitmap wbmp = img.Source as WriteableBitmap;
            return new Action<VideoFrame>(frame =>
            {
                var rect = new Int32Rect(0, 0, frame.Width, frame.Height);
                wbmp.WritePixels(rect, frame.Data, frame.Stride * frame.Height, frame.Stride);
            });
        }
        private void SetupWindow(PipelineProfile pipelineProfile, out Action<VideoFrame> depth, out Action<VideoFrame> color, out Action<VideoFrame> ir1, out Action<VideoFrame> ir2)
        {
            using (var p = pipelineProfile.GetStream(Intel.RealSense.Stream.Depth).As<VideoStreamProfile>())
                imgDepth.Source = new WriteableBitmap(p.Width, p.Height, 96d, 96d, PixelFormats.Rgb24, null);
            depth = UpdateImage(imgDepth);

            using (var p = pipelineProfile.GetStream(Intel.RealSense.Stream.Color).As<VideoStreamProfile>())
                imgColor.Source = new WriteableBitmap(p.Width, p.Height, 96d, 96d, PixelFormats.Rgb24, null);
            color = UpdateImage(imgColor);

            using (var p = pipelineProfile.GetStream(Intel.RealSense.Stream.Infrared).As<VideoStreamProfile>())
                imgColor.Source = new WriteableBitmap(p.Width, p.Height, 96d, 96d, PixelFormats.Rgb24, null);
            ir1 = UpdateImage(imgIRleft);

            using (var p = pipelineProfile.GetStream(Intel.RealSense.Stream.Infrared).As<VideoStreamProfile>())
                imgColor.Source = new WriteableBitmap(p.Width, p.Height, 96d, 96d, PixelFormats.Rgb24, null);
            ir2 = UpdateImage(imgIRright);


            //using (var p = pipelineProfile.GetStream(Stream.Color).As<VideoStreamProfile>())
            //    ip.Source = new WriteableBitmap(p.Width, p.Height, 96d, 96d, PixelFormats.Rgb24, null);
            //iproc = UpdateImage(ip);
        }

        public MainWindow()
        {
            InitializeComponent();

            RunProcess();
        }

        #endregion

    }
}
