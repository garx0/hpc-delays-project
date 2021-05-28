# hpc-delays-project

Attempt to parallelize https://github.com/garx0/graduate-work/tree/master/delaytool project with OpenMP to speed up longer calculations of delay estimates in real-time network.

Project presentation with its description: https://docs.google.com/presentation/d/1w3GfoYDi8lSjZY38IGjVViFYd_6JAO68ka_OwdLPquE

Works for schemes (`-s`) `oqp` and `oqc` now.

Examples:

`cd build`, then:

`./delaytool ../experiments/vlconfigs/msggen1/3s12e_final_vls_1.xml ../experiments/test.xml -f 4 -s oqc -c 53 -t 2`

`./delaytool ../experiments/vlconfigs/msggen1/3s12e_final_vls_1.xml ../experiments/test.xml -f 4 -s oqp -t 2`

---

`-t` or `--threads` - number of threads

`-t 1` - multi-thread algorithm run on single thread

`-t 0` - original single-thread algorithm which was not designed for parallelization (faster that `-t 1`)
