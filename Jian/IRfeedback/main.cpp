
//=================================================================================-----
//== NaturalPoint 2010
//== Camera Library SDK Sample
//==
//== This sample brings up a connected camera and displays it's output frames.
//=================================================================================-----

#include "supportcode.h"       //== Boiler-plate code for application window init ===---
#include "cameralibrary.h"     //== Camera Library header file ======================---
#include "cameramanager.h"
#include <windows.h>
#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <math.h>

#include "calibration.h"
#include "main.h"
#include "vision_tcp.h"

using namespace CameraLibrary; 
using namespace std;


int main(int argc, char* argv[])
{
	VisionTCP IRvisionTCP;
	//Initialize TCPIP
	if (!TESTMODE){
		
		if (int init = IRvisionTCP.Init()) {
			OUTPUT("No TCPIP connection available!");
			//return init;
		}
	}


	//== For OptiTrack Ethernet cameras, it's important to enable development mode if you
	//== want to stop execution for an extended time while debugging without disconnecting
	//== the Ethernet devices.  Lets do that now:

	CameraLibrary_EnableDevelopment();

	//== Initialize connected cameras ==========----
	
    CameraManager::X().WaitForInitialization();

    //== Get a connected camera ================----

    Camera *camera = CameraManager::X().GetCamera();

    //== If no device connected, pop a message box and exit ==--

    if(camera==0)
    {
        MessageBox(0,"Please connect a camera","No Device Connected", MB_OK);
        return 1;
    }

	//== Determine camera resolution to size application window ==----
	
    int cameraWidth  = camera->Width();
    int cameraHeight = camera->Height();
	int i, j ;
	char buff[3000];
	// Define the data buffer:
	//double posbuff[framecnt][NUM_MARKERS*2+1];
	double calibuff[CALIBRA_NUM][2];

 	//== Open the application window =============================----
		
    if (TESTMODE)
	{
		if (!CreateAppWindow("Camera Library SDK - Sample",cameraWidth,cameraHeight,32,gFullscreen))  return 0;
	

    //== Create a texture to push the rasterized camera image ====----

    //== We're using textures because it's an easy & cpu light 
    //== way to utilize the 3D hardware to display camera
    //== imagery at high frame rates
		
	}
	Surface Texture(cameraWidth, cameraHeight);

	//if (TESTMODE == 1){
		
		Bitmap * framebuffer = new Bitmap(cameraWidth, cameraHeight, Texture.PixelSpan()*4,
								   Bitmap::ThirtyTwoBit, Texture.GetBuffer());
	//}
	
    //== Set Video Mode ==--

    //camera->SetVideoType(SegmentMode);
	camera->SetVideoType(ObjectMode);
    
	//Set camera frame rate
	camera->SetFrameRate(FRAMERATE);

	//==Set camera exposure
	camera->SetExposure(25);

    //== Start camera output ==--

    camera->Start();
	Core::DistortionModel distortion;
	distortion.Distort = true;
	camera->GetDistortionModel(distortion);

    //== Turn on some overlay text so it's clear things are     ===---
    //== working even if there is nothing in the camera's view. ===---
	if (TESTMODE == 1){
		camera->SetTextOverlay(true);

		camera->SetMarkerOverlay(true);
	}

	/*************************************************************************************************/
    //== Ok, start main loop.  This loop fetches and displays   ===---
    //== camera frames.                                         ===---
	int flag = 0;
	double preX[NUM_MARKERS], preY[NUM_MARKERS];
	double prew_X, prew_Y;
	int preFID = 0;
	double min_interval;
	double w_X[NUM_MARKERS], w_Y[NUM_MARKERS];
	double tt, v, w[NUM_MARKERS], w_ave;
	char index;
	char temp[300];
	int y = 1;
	
	int runcnt = 0;

	for (i = 0; i < 5; i++){
		IRvisionTCP.Send(0, 0.0 , 0.0);
	}

    while(1)
	{	  
		//while (!keys[VK_SPACE]) { }
          
        //The for loop here is just for delay a little while.
		for (i=1 ; i<100 ; i++) ;
		//---=== Fetch a new frame from the camera ===---
        Frame *frame = camera->GetFrame();
		
        if(frame)
        {
		
				//x = frame->TimeStamp();
			if ((frame->FrameID() - preFID) == 0) continue;
			
			//QueryPerformanceCounter(&t2);
            //== Ok, we've received a new frame, lets do something
            //== with it.

            //== Lets have the Camera Library raster the camera's
            //== image into our texture.
		
            frame->Rasterize(framebuffer);

			//camera->AttachModule(new cModuleLabeler());

            //== Display Camera Image ============--

            if (TESTMODE){
				if(!DrawGLScene(&Texture)) break;
			}

            //== Escape key to exit application ==--

            if (keys[VK_ESCAPE])
                break;

			//if (keys[VK_SPACE]) //Space key to reset the markers index number
            //    flag = 0;
			
			if(frame->ObjectCount()>0)
			{
				if (frame->ObjectCount() == NUM_MARKERS)
				{
					if (abs(frame->Object(0)->X()) > cameraWidth) 
						continue;

					runcnt++;  

					if ((runcnt > framecnt) && (!TESTMODE) ) 
						break;
				}
				else 
				{
					if (!TESTMODE) 
					{
						OUTPUT("The # of markers is not what you want!");
						continue;
					}
				}

				cObject * object;
				//cObject *object = (cObject *)malloc(frame->ObjectCount() * sizeof (cObject *));
				//object = frame->Object(0);
			    //camera->MarkerOverlay();
				//elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
				

				for (i = 0; i < frame->ObjectCount(); i++)
				{
					/*if(!(NUM_MARKERS == frame->ObjectCount())) 
					{
						MessageBox(0,"The # of markers is not what you want!","Oh!", MB_OK);
						return 1;
					}*/
					tt = frame->TimeStamp();

					if(!TESTMODE)
					{
						
						if (flag == 0) { // Reset the markers previous index information
						
							for (j = 0; j < NUM_MARKERS; j++){
								object = frame->Object(j);
								preX[j] = object->X();
								preY[j] = object->Y();																		
							}
							preFID = frame->FrameID();
							flag = 1;
							continue;
						}
								
						object = frame->Object(i);

						min_interval = abs(object->X() - preX[i]) + abs(object->Y() - preY[i]);
						index = i;
						

						for (j = 0; j < frame->ObjectCount(); j++)
						{
							//if ( (abs(object->X() - preX[j]) < err) && (abs(object->Y() - preY[j]) < err) ) 
							//{
							//	index = j;
								//preX[index] = object->X();  preY[index] = object->Y();
							//	break;
							//}
							if ((abs(object->X() - preX[j]) + abs(object->Y() - preY[j])) < min_interval) 
							{
								min_interval = abs(object->X() - preX[j]) + abs(object->Y() - preY[j]);
								index = j;
								//preX[index] = object->X();	preY[index] = object->Y();
							}
						}

						prew_X = a11*preX[index] + a12*preY[index] + a13;
						prew_Y = a21*preX[index] + a22*preY[index] + a23;

						preX[index] = object->X();  preY[index] = object->Y();

						/*for (j = 0; j < NUM_MARKERS; j++){
								object = frame->Object(j);
								preX[j] = object->X();
								preY[j] = object->Y();																		
						}*/
					
					//}
					//object = frame->Object(i);

					//Calculate the calibrated position in the world frame
						w_X[index] = a11*object->X() + a12*object->Y() + a13;
						w_Y[index] = a21*object->X() + a22*object->Y() + a23;

						v = ((w_X[index] - prew_X)*(w_X[index] - prew_X) + (w_Y[index] - prew_Y)*(w_Y[index] - prew_Y));
						//v = sqrt(v)/((frame->FrameID() - preFID)/FRAMERATE);
						v = sqrt(v)*FRAMERATE/(frame->FrameID() - preFID);
											
						if (index == 0) w[0] = v/r1;
						else if (index == 1) w[1] = v/r2;
						else if (index == 2) w[2] = v/r3;
						else if (index == 3) w[3] = v/r4;

					//double w_Xtemp, w_Ytemp;
					//w_X = a11*object->X() + a12*object->Y() + a13*(object->X())*(object->X()) + a14*(object->Y())*(object->Y()) + a15*(object->X())*(object->Y()) + a16;
					//w_Y = a21*object->X() + a22*object->Y() + a23*(object->X())*(object->X()) + a24*(object->Y())*(object->Y()) + a25*(object->X())*(object->Y()) + a26;

					}

					if (TESTMODE){
						object = frame->Object(i);
						w_X[0] = a11*object->X() + a12*object->Y() + a13;
						w_Y[0] = a21*object->X() + a22*object->Y() + a23;
						//sprintf(buff, "frameRate:%d frame# %d (w:%d,h:%d), Object index:%d,m#:%d, f: %f %f w: %f %f",
						//	camera->FrameRate(), frame->FrameID(),cameraWidth, cameraHeight, index ,i,object->X(), object->Y(), w_X, w_Y);
						sprintf_s(buff, "frame# %d , f: %f %f w: %f %f",
							frame->FrameID(), object->X(), object->Y(), w_X[0], w_Y[0]);
						OUTPUT(buff);

						FILE * fp;
						fp = fopen("camera_frame.txt","a");
						
						fprintf(fp, "%f %f 1\n", object->X(), object->Y());
						
						fclose(fp);
						
						if (frame->ObjectCount() != CALIBRA_NUM) continue;
						calibuff[i][0] = object->X();
						calibuff[i][1] = object->Y();

					}
										
				}// 'for' loop of calculating all the markers

				w_ave = 0.0;
				for (i = 0; i < NUM_MARKERS; i++) w_ave += w[i];
				w_ave = w_ave/NUM_MARKERS;

				if (runcnt % 1 == 0 && runcnt > 0 && !TESTMODE){
					IRvisionTCP.Send(frame->FrameID(), w_ave , tt);
					OUTPUT("send!");
				}
				
				preFID = frame->FrameID();

				//OUTPUT markers data
				if (!TESTMODE)
				{
					

					//This part is for output the data to the Output window
					sprintf_s(buff, "tt: %f frame# %d , f: %f %f w: #0 %f %f ;",
							tt, frame->FrameID(), preX[0], preX[0], w_X[0], w_Y[0]);
					for (i = 1; i < NUM_MARKERS; i++)
					{
						sprintf_s(temp, " #%d %f %f",
							i, w_X[i], w_Y[i]);
						strcat_s(buff, temp);
					}
					OUTPUT(buff);
					
					//write the position data to the buffer which will be written into text file
					for (i = 0; i < NUM_MARKERS; i++)
					{
						//posbuff[runcnt-1][i*2] = w_X[i];
						//posbuff[runcnt-1][i*2+1] = w_Y[i];
					}
					//posbuff[runcnt-1][NUM_MARKERS*2] = frame->FrameID();
					
				}
				else
				{
					FILE * fp;
					fp = fopen("camera_frame.txt","w");
						
					for (i = 0; i < CALIBRA_NUM; i++)
					{
						fprintf(fp, "%f %f 1 \n", calibuff[i][0], calibuff[i][1]);
					}
												
					fclose(fp);
				}
				
			}

			//OUTPUT("sj");
            //== Release frame =========--
            frame->Release();
			if (TESTMODE) 
			{
				OUTPUT("---------------------------------------------------------------------------------------------");
				FILE * fp;
						fp = fopen("camera_frame.txt","w");
						
						fprintf(fp, "\n");
						
						fclose(fp);
			}
		}

	    //Sleep(1);

        //== Service Windows Message System ==--

        if(!PumpMessages())
            break;

		//free(frame);

    }//while

	for (i = 0; i<3; i++){
		IRvisionTCP.Send(1, 1.0 , 1.0);
	}

	if (!TESTMODE)
	{
		FILE * fp;
		fp = fopen(FILENAME,"w");

		for (i = rid; i < framecnt; i++)
		{
			//for (j = 0; j < NUM_MARKERS*2; j++)	fprintf(fp,"%f ",posbuff[i][j]);			
			//fprintf(fp, "%d\n", int(posbuff[i][NUM_MARKERS*2]));
		}
		fclose(fp);
	}
    //== Close window ==--

    if(TESTMODE) CloseWindow();

    //== Release camera ==--

    camera->Release();

    //== Shutdown Camera Library ==--

    CameraManager::X().Shutdown();

    //== Exit the application.  Simple! ==--

	return 1;
}

