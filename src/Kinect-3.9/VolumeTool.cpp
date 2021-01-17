#include "VolumeTool.h"


#include <iostream>
#include <Math/Math.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/Box.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Geometry/PCACalculator.h>
#include <Geometry/OutputOperators.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/ToolManager.h>
#include <Vrui/DisplayState.h>
#include <Kinect/Camera.h>

#include <iomanip>
#include <fstream>

#include "RawKinectViewer.h"

/**********************************
Static elements of class VolumeTool:
**********************************/

VolumeToolFactory* VolumeTool::factory=0;

/********************************
Methods of class VolumeTool:
********************************/

VolumeToolFactory* VolumeTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new VolumeToolFactory("VolumeTool","Collect volume",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(2);
	factory->setButtonFunction(0,"Start Volume");
	factory->setButtonFunction(1,"End Volume");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

VolumeTool::VolumeTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment)
	{
	}

VolumeTool::~VolumeTool(void)
	{
	}

const Vrui::ToolFactory* VolumeTool::getFactory(void) const
{
return factory;
}


void VolumeTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState) // Button was just pressed
		{
		if(buttonSlotIndex==0){
			std::cout << "Volume start" << std::endl;
            p0=Point(application->calcImagePoint(getButtonDeviceRay(0)).getComponents());
		    dragging=true;
			button2 = false;
		}

		if(buttonSlotIndex==1){
			std::cout << "Volume end" << std::endl;
			button2 = true;
		}
		}
	else // Button was just released
		{
			std::cout << "button released" << std::endl;
			if(!button2)
			{
				//dragging=false;

				//application->requestAverageFrame(0);

				printf("p0 = %f, %f\n", p0[0], p0[1]);
				printf("p1 = %f, %f\n", p1[0], p1[1]);
				//printf("min1 = %d, min2 = %d, max1 = %d, max2 = %d\n", min[0], min[1], max[0], max[1]);

				RawKinectViewer::CPoint imagePoint0 =application->getDepthImagePoint(p0);
				RawKinectViewer::CPoint imagePoint1 =application->getDepthImagePoint(p1);

				RawKinectViewer::CPoint worldPoint0 = application->intrinsicParameters.depthProjection.transform(imagePoint0);
				RawKinectViewer::CPoint worldPoint1 = application->intrinsicParameters.depthProjection.transform(imagePoint1);
				std::cout<<std::setw(20)<<worldPoint0<<std::endl;
				std::cout<<std::setw(20)<<worldPoint0[2]<<std::endl;
				std::cout<<std::setw(20)<<worldPoint1<<std::endl;

				xValEnd = floor(p1[0]);
				yValEnd = floor(p1[1]);

				//TODO: Add a case statement or if conditions for each possible starting corner
				
				xSize = floor(p1[0] - p0[0]);
				ySize = floor(p0[1] - p1[1]);
				std::cout << "xSize = " << xSize << ", ySize = " << ySize << std::endl;

				depthArray = new int*[xSize];
				for(int i = 0; i < xSize; i++)
				{
					depthArray[i] = new int[ySize];
				}

				for(double xValStart = p0[0]; xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = p0[1]; yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = floor(xValEnd - xValStart);
						int yPos = floor(yValStart - yValEnd); 

						RawKinectViewer::CPoint imagePosition =application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);
						depthArray[xPos][yPos] = worldPosition[2];
						//printf("depthArray[%d][%d] = %f\n", xPos, yPos, worldPosition[2]);
					}
					
				}
				
				//application->averageFrameValid=false;
				//application->depthPlaneValid=false;
			}
			else
			{
				depthArray2 = new int*[xSize];
				for(int i = 0; i < xSize; i++)
				{
					depthArray2[i] = new int[ySize];
				}

				for(double xValStart = p0[0]; xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = p0[1]; yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = floor(xValEnd - xValStart);
						int yPos = floor(yValStart - yValEnd); 

						RawKinectViewer::CPoint imagePosition =application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);
						depthArray2[xPos][yPos] = worldPosition[2];
					}
				}

				double volume = 0;

				for(int i = 0; i < xSize; i++)
				{
					for(int j = 0; j < ySize; j++)
					{
						if(depthArray2[i][j] < depthArray[i][j])
						{
							double difference = depthArray2[i][j] - depthArray[i][j];
							printf("difference = %f\n", difference);
							volume = volume + difference;
						}
					}
				}
				printf("volume = %f\n", volume);
			}
		}
	}

void VolumeTool::frame(void)
	{
	if(dragging)
		{
		/* Get the current rectangle point: */
		p1=Point(application->calcImagePoint(getButtonDeviceRay(0)).getComponents());
		}
	}

void VolumeTool::display(GLContextData& contextData) const
	{
	if(dragging)
		{
		glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
		glDisable(GL_LIGHTING);
		glLineWidth(1.0f);
		
		/* Go to navigation coordinates: */
		glPushMatrix();
		const Vrui::DisplayState& displayState=Vrui::getDisplayState(contextData);
		glLoadMatrix(displayState.modelviewNavigational);
		
		/* Draw the current rectangle: */
		glColor3f(0.0f,0.333f,0.0f);
		glBegin(GL_LINE_LOOP);
		glVertex3d(p0[0],p0[1],0.01);
		glVertex3d(p1[0],p0[1],0.01);
		glVertex3d(p1[0],p1[1],0.01);
		glVertex3d(p0[0],p1[1],0.01);
		glEnd();
		
		glPopMatrix();

		glPopAttrib();
		}
	}
