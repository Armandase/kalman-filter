# Kalman Filter

C++ Kalman filter implementation that connects to a UDP stream of inertial measurement unit (IMU) data, estimates the current position.

Trajectory visualization with GNUplot.

## Project Highlights

- Takes IMU input data from an UDP client (default `127.0.0.1:4242`) using `UdpClient`.
- Update state estimation with a configurable `KalmanFilter` class (backed by [Eigen](https://eigen.tuxfamily.org/)).
- Saves position estimate and variance to be plotted in 2D or 3D.


## Prerequisites

Make sure the following tools are installed:

- A C++17 compiler (GCC or Clang)
- [CMake](https://cmake.org/) 3.16+
- [Eigen](https://eigen.tuxfamily.org/) (downloaded automatically through CMake)
- [GNUplot](http://www.gnuplot.info/)

On Debian/Ubuntu based systems you can install GNUplot with:

```bash
sudo apt-get install gnuplot
```

## Building

```bash
cmake -S . -B build
cmake --build build --config Release
```

## Running the Filter

1. Start UDP data source flux that publishes IMU packets on port `4242`. You can use `imu-sensor-stream-linux` or your own sensor simulator.
2. Launch the client:

   ```bash
   ./build/ft_kalman
   ```

### Output Files

- `estimated_positions.txt`: one estimated position (X, Y, Z) per line.
- `variance.txt`: variance of each axis for the corresponding time step.


## Visualising with GNUplot

All commands below pause until you close the plot window.

### 3D Trajectory

```bash
gnuplot -persist -e "splot 'estimated_positions.txt' with linespoints title 'Estimated Position'; pause -1"
```

### 2D XY Projection

```bash
gnuplot -persist -e "plot 'estimated_positions.txt' using 1:2 with linespoints title 'Estimated XY'; pause -1"
```

### Dual-Axis Variance Plots

#### X axis (blue vs red)

```bash
gnuplot -persist -e "set title 'X Position and X Variance'; \
    set xlabel 'Time step'; \
    set ylabel 'X Position'; \
    set y2label 'X Variance'; \
    set y2tics; \
    plot 'estimated_positions.txt' using 0:1 with lines title 'X Position' axis x1y1 lc rgb 'blue', \
         'variance.txt' using 0:1 with lines title 'X Variance' axis x1y2 lc rgb 'red'; \
    pause -1"
```

#### Y axis (green vs magenta)

```bash
gnuplot -persist -e "set title 'Y Position and Y Variance'; \
    set xlabel 'Time step'; \
    set ylabel 'Y Position'; \
    set y2label 'Y Variance'; \
    set y2tics; \
    plot 'estimated_positions.txt' using 0:2 with lines title 'Y Position' axis x1y1 lc rgb 'green', \
         'variance.txt' using 0:2 with lines title 'Y Variance' axis x1y2 lc rgb 'magenta'; \
    pause -1"
```

#### Z axis (blue vs red)

```bash
gnuplot -persist -e "set title 'Z Position and Z Variance'; \
    set xlabel 'Time step'; \
    set ylabel 'Z Position'; \
    set y2label 'Z Variance'; \
    set y2tics; \
    plot 'estimated_positions.txt' using 0:3 with lines title 'Z Position' axis x1y1 lc rgb 'blue', \
         'variance.txt' using 0:3 with lines title 'Z Variance' axis x1y2 lc rgb 'red'; \
    pause -1"
```

Alternatively, run one of the helper scripts (`2d_plot.sh`, `3d_plot.sh`, `variance_3d_pos.sh`, etc.) to spawn preconfigured charts.

## Bibliographie

- https://www.ferdinandpiette.com/blog/2011/04/exemple-dutilisation-du-filtre-de-kalman/
- https://pagespro.isae-supaero.fr/IMG/pdf/introKalman_vf_2008.pdf
- https://apps.dtic.mil/sti/tr/pdf/ADA024377.pdf

## Contributors

This work was done collaboratively by:
- **[Dboire](https://github.com/Dboire9)**
- **[Armandase](https://github.com/Armandase)**