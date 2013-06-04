The file you need to modify is almost all SampleLoopTask_.C 

The final files for each of the control objectives are

- One step back stepping (stabilization only): SampleLoopTask_oneBack.C

- speed control using 1step BS: SampleLoopTask_thetahDCtrl_oneBack.C

- speed control using Feedback Linearization: SampleLoopTask_thetahDCtrl_FL.C

- position control using FL: SampleLoopTask_thetahCtrl_FL.C

- all three above with speed control w/ FL: SampleLoopTask_total.C


Some other previous important files are

- Fabio's control: SampleLoopTask_fabio_final.C

- Olfati's control: SampleLoopTask_Olfati.C

- Using 2step standard BS: SampleLoopTask_stdBack6.C

However, except those five files up above, others including all other unliated files may need to be modified accordingly. For example, X_OFFSET and the way to get feedback values and so on. So to do that, see the latest files. Also, for the previous methods, ways I used and tried, see [file_name]_old.C for the five files and some other old files.




