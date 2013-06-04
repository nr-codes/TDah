
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

#include "calibration.h"
//#include <cv.h>
//#include <cxcore.h>
//#include <highgui.h>

//using namespace cv;
//using namespace std;

#define NUM_MARKERS 4   //# of the tracking markers
#define CALIBRA_NUM 98  //This is the number of total dots on the calibration board

#define TESTMODE 1 // Testmode is for testing and calibration 
#define FRAMERATE 250
#define Time_to_record 20 //in second
#define FILENAME "TOM3.txt" //The name for output file

#define rid 100   //Since at the beginning the data seems to be a little unstable, so get 'rid' of the first # data.

#define framecnt int(FRAMERATE*Time_to_record + rid)
#define err 20

using namespace CameraLibrary; 
using namespace std;

/*void CreateMyMultilineTextBox() {
        // Create an instance of a TextBox control
        TextBox __gc *textBox1 = new TextBox();

        // Set the Multiline property to true.
        textBox1->Multiline = true;
        // Add vertical scroll bars to the TextBox control.
        textBox1->ScrollBars = ScrollBars::Vertical;
        // Allow the RETURN key to be entered in the TextBox control.
        textBox1->AcceptsReturn = true;
        // Allow the TAB key to be entered in the TextBox control.
        textBox1->AcceptsTab = true;
        // Set WordWrap to True to allow text to wrap to the next line.
        textBox1->WordWrap = true;
        // Set the default text of the control.
        textBox1->Text = S"Welcome!";
    };*/


int main(int argc, char* argv[])
{
	cout << "jiangediaoda";
	//== For OptiTrack Ethernet cameras, it's important to enable development mode if you
	//== want to stop execution for an extended time while debugging without disconnecting
	//== the Ethernet devices.  Lets do that now:

	/*ofstream myfile;
	myfile.open ("example.txt");
    myfile << "Writing this to a file.\n";
    myfile.close();*/

	CameraLibrary_EnableDevelopment();

	//printf("sjja;sjf\n");

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
	//int framecnt = FRAMERATE*Time_to_record;
	double posbuff[framecnt][NUM_MARKERS*2+1];
	double calibuff[CALIBRA_NUM][2];
	//char TESTMODE;

	//TESTMODE = 0; 

	//LARGE_INTEGER frequency;        // ticks per second
   // LARGE_INTEGER t1, t2;           // ticks
    //double elapsedTime;

    // get ticks per second
    //QueryPerformanceFrequency(&frequency);
    //QueryPerformanceCounter(&t1);

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

    //== Ok, start main loop.  This loop fetches and displays   ===---
    //== camera frames.                                         ===---
	int flag = 0;
	float preX[NUM_MARKERS], preY[NUM_MARKERS];
	float min_interval;
	double w_X[NUM_MARKERS], w_Y[NUM_MARKERS];
	double tt;
	char index;

	//cvNamedWindow("aaa");

	int runcnt = 0;

    while(1)
	{	  
		//while (!keys[VK_SPACE]) { }
          
        //== Fetch a new frame from the camera ===---
		
		
        Frame *frame = camera->GetFrame();
		
        if(frame)
        {
		

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
					if ((runcnt > framecnt) && (!TESTMODE) ) break;
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
							flag = 1;
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

						/*FILE * fp;
						fp = fopen("camera_frame.txt","a");
						
						fprintf(fp, "%f %f 1\n", object->X(), object->Y());
						
						fclose(fp);
						*/
						if (frame->ObjectCount() != CALIBRA_NUM) continue;
						calibuff[i][0] = object->X();
						calibuff[i][1] = object->Y();

					}
										
				}

				
				//OUTPUT markers data
				if (!TESTMODE)
				{
					char temp[300];

					sprintf_s(buff, "tt: %f frame# %d , f: %f %f w: #0 %f %f ;",
							tt, frame->FrameID(), preX[0], preX[0], w_X[0], w_Y[0]);
					for (i = 1; i < NUM_MARKERS; i++)
					{
						sprintf_s(temp, " #%d %f %f",
							i, w_X[i], w_Y[i]);
						strcat_s(buff, temp);
					}
					OUTPUT(buff);
					
					
					for (i = 0; i < NUM_MARKERS; i++)
					{
						posbuff[runcnt-1][i*2] = w_X[i];
						posbuff[runcnt-1][i*2+1] = w_Y[i];
					}
					posbuff[runcnt-1][NUM_MARKERS*2] = frame->FrameID();
					
				}
				else
				{
					FILE * fp;
					fp = fopen("camera_frame.txt","w");
						
					for (i = 0; i < CALIBRA_NUM; i++)
					{
						fprintf(fp, "%f %f 1\n", calibuff[i][0], calibuff[i][1]);
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
				/*FILE * fp;
						fp = fopen("camera_frame.txt","w");
						
						fprintf(fp, "\n");
						
						fclose(fp);*/
			}
		}

	    //Sleep(0);

        //== Service Windows Message System ==--

        if(!PumpMessages())
            break;

		//free(frame);
    }

	if (!TESTMODE)
	{
		FILE * fp;
		fp = fopen(FILENAME,"w");

		for (i = rid; i < framecnt; i++)
		{
			for (j = 0; j < NUM_MARKERS*2; j++)	fprintf(fp,"%f ",posbuff[i][j]);			
			fprintf(fp, "%d\n", int(posbuff[i][NUM_MARKERS*2]));
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

