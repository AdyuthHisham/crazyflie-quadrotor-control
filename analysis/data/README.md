`c_sim_simulation.csv`: log written by the course's desktop FreeRTOS simulation while running `firmware/filter.c` and
`firmware/controller.c` (2000 rows, 50 ms apart, 99.5 s). Columns are the course template's logging format (actual and estimated
roll/pitch/yaw, references, motor %, four log values). Produced locally on 2026-11-23 with `./crazyflie`; see `firmware/README.md`.
The plant in this run is the course template's dynamic model, not the real Crazyflie.
