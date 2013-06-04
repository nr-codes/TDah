# These files are for record. As time progresses, there have been lots of changes in the code.  So I'm not sure if I can use it intact without updating the later changes.

SampleLoopTask_org.C : don't know when this was used, but the oldest file.

2/14/11
SampleLoopTask_old1.C : before chaning #DEFINE constants to const constants



SampleLoopTask_fabio1.C: W/O acceleration control part.  Just used the direct motor modeling, but worked well.

SampleLoopTask_innerloop1.C: W/ the current command feedforward, added vel, acc corrective term.

SampleLoopTask_innerloop2.C: Just worked well with only velocity tracking.

SampleLoopTask_acc_innerloop1.C
SampleLoopTask_acc_innerloop2.C : These are for acceleration tracking before implementing higher control

SampleLoopTask_old2.C : for record

SampleLoopTask_frSweep.C: for frequency sweep.

SampleLoopTask_fabio_final: for frequency sweep.



4/14/11
SampleLoopTask_fabio_automaticUpdate.C : fabio_final + automatic eq. pt update.

sampleLoopTask_fabio_auto_Linear.c: fabio_final + automatic eq. pt update. + linear control


5/18/11
SampleLoopTask_stdBack4.C: standard backstepping final (used in 85 deg setup)

5/xx/11
SampleLoopTask_stdBack5.C: standard backstepping, used in 85 deg setup, x-center of the object position is hard coded.

7/28/11
SampleLoopTask_stdBack6.C: standard backstepping final (/w correction in the h term)

11/28/11
SampleLoopTask_Olfati.C: Olfati-Saber's control law implementation.

11/30/11
SampleLoopTask_fabio_final1.C: In order to check if the system is OK after backing the camera up.
Unlike ..._fibio_final.c, this file uses the hardcoding x_offset for the center position of the object disk.
Also, the tiltinig degree in GRAV was set to 85.2 degree.

