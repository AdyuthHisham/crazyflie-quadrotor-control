# Firmware (C, FreeRTOS)

Our files from the course project 5 repository (FreeRTOS tasks run on a desktop with the course's simulated Crazyflie).
The C was generated from MATLAB code with a MATLAB-to-C conversion plugin (per the author) and then edited in the commits below; it was not written by hand. These files are **not a complete program**. `main.c`, the dynamic model, `constants.*`, `helpers.*`, the reference generator and the
FreeRTOS kernel (v9.0.0, GPL-2 with linking exception) come from the course template and are not included; the headers
here only declare the parameter structs the template passes in.

| File | What it does |
|---|---|
| `filter.c`, `filter.h` | Complementary filter task, 5 ms, roll and pitch from accelerometer + gyro, semaphore-protected reads and writes |
| `controller.c`, `controller.h` | 100 Hz LQ controller task: reads gyro, estimate and reference under semaphores, applies gain K, writes motor PWM |

To rebuild the simulation behind `analysis/data/c_sim_simulation.csv` you need the course template (Makefile, `main.c`,
FreeRTOS POSIX port): copy these four files over the template's, run `make`, then `./crazyflie`. It takes about 100 s and
writes `simulation.csv`.
