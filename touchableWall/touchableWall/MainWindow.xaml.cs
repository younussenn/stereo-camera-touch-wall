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
using Image = System.Windows.Controls.Image;
using PixelFormat = System.Drawing.Imaging.PixelFormat;
using Point = System.Drawing.Point;

namespace touchableWall
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {


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

        Action<VideoFrame> updateDepth, updateColor, updateIR1,updateIR2;


        public const int _limit = 80;

        int resolutionW = 480, resolutionH = 270, FPS = 60;
        int screenWidth, screenHeight;
        bool wallDistanceIsCalibrated = false;
        bool isCalibrated = false, isCalibrating = false;
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
                advancedDevice.JsonConfiguration = File.ReadAllText(@"DefaultConfig_D435.json");
                selectedDevice = pipelineProfile.Device;

                #region Field Of View Info

                float[] dfov, cfov,irfov;

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
                SetupWindow(pipelineProfile, out updateDepth, out updateColor,out updateIR1,out updateIR2);

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

        private void wallDistanceCalibration(DepthFrame depthFrame)
        {
            try
            {



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
        }


        #region xmlData





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


        #endregion



    }
}
