using Intel.RealSense;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace touchableWall
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {


        private Pipeline pipeline;
        private PipelineProfile pipelineProfile;
        private Colorizer colorizer;
        private Device selectedDevice;
        private CustomProcessingBlock block;
        private CancellationTokenSource tokenSource = new CancellationTokenSource();
        private DecimationFilter decimationFilter;
        private ThresholdFilter thresholdFilter;
        private HoleFillingFilter holeFilter;
        private SpatialFilter spatialFilter;
        private TemporalFilter temporalFilter;
        private Align align_to;


        public const int esikdeger = 80;

        int resolutionW = 480, resolutionH = 270, FPS = 60;

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

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
                    
            }


        }



        public MainWindow()
        {
            InitializeComponent();
        }
    }
}
