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
			if(!button2)
			{
				dragging=false;

				printf("top left corner:     p0 = %f, %f\n", p0[0], p0[1]);
				printf("bottom right corner: p1 = %f, %f\n", p1[0], p1[1]);

				RawKinectViewer::CPoint imagePoint0 = application->getDepthImagePoint(p0);
				RawKinectViewer::CPoint imagePoint1 = application->getDepthImagePoint(p1);

				RawKinectViewer::CPoint worldPoint0 = application->intrinsicParameters.depthProjection.transform(imagePoint0);
				RawKinectViewer::CPoint worldPoint1 = application->intrinsicParameters.depthProjection.transform(imagePoint1);

				std::cout<<std::setw(20)<<worldPoint0<<std::endl;
				std::cout<<std::setw(20)<<worldPoint1<<std::endl;

				xValEnd = floor(p1[0]);
				yValEnd = floor(p1[1]);

				//TODO: Add a case statement or if conditions for each possible starting corner
				
				xSize = (floor(p1[0]) - floor(p0[0]));
				ySize = (floor(p0[1]) - floor(p1[1]));
				std::cout << "xSize = " << xSize << ", ySize = " << ySize << std::endl;

				//struct volumetricPoint testArray[xSize][ySize];

				depthArray = new struct volumetricPoint *[xSize];
				for(int i = 0; i < xSize; i++)
				{
					depthArray[i] = new struct volumetricPoint[ySize];
				}
				//depthArray = new struct volumetricPoint[xSize][ySize];
				
				std::ofstream outFile;
				outFile.open("initialPoints.csv");

				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;
						
						//printf("xStart = %f, yStart = %f\n", xValStart, yValStart);

						int xPos = ((xValEnd - xValStart) -1); //*10)-1;
						int yPos = ((yValStart - yValEnd) -1); //*10)-1;


						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						//printf("depth image point = %f %f %f\n", imagePosition[0], imagePosition[1], imagePosition[2]);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);
						depthArray[xPos][yPos].volumePoint = worldPosition;
						outFile <<  worldPosition[0] << "," << worldPosition[1] << "," << worldPosition[2] << "\n";

						//printf("testArray[%d][%d] -> x = %f, y = %f, z = %f\n", xPos, yPos, depthArray[xPos][yPos].volumePoint[0], depthArray[xPos][yPos].volumePoint[1], depthArray[xPos][yPos].volumePoint[2]);
					}
					
				}
				outFile.close();
			}
			else
			{
				depthArray2 = new struct volumetricPoint *[xSize];
				for(int i = 0; i < xSize; i++)
				{
					depthArray2[i] = new struct volumetricPoint[ySize];
				}

				std::ofstream outFile2;
				outFile2.open("secondPoints.csv");

				for(int xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(int yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = (xValEnd - xValStart)-1;
						int yPos = (yValStart - yValEnd)-1; 

						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);
						depthArray2[xPos][yPos].volumePoint = worldPosition;
						outFile2 <<  worldPosition[0] << "," << worldPosition[1] << "," << worldPosition[2] << "\n";

					}
				}
				outFile2.close();
								
				double volume = 0;

				for(int i = 0; i < xSize-1; i++) //xSize
				{
					for(int j = 0; j < ySize-1; j++) //ySize
					{
						double x1Before = depthArray[i][j].volumePoint[0];
						double y1Before = depthArray[i][j].volumePoint[1];
						double z1Before = depthArray[i][j].volumePoint[2];

						double x2Before = depthArray[i+1][j].volumePoint[0];
						double y2Before =  depthArray[i+1][j].volumePoint[1];
						double z2Before = depthArray[i+1][j].volumePoint[2];

						double x3Before = depthArray[i][j+1].volumePoint[0];
						double y3Before =  depthArray[i][j+1].volumePoint[1];
						double z3Before = depthArray[i][j+1].volumePoint[2];

						double x4Before = depthArray[i+1][j+1].volumePoint[0];
						double y4Before =  depthArray[i+1][j+1].volumePoint[1];
						double z4Before = depthArray[i+1][j+1].volumePoint[2];

						double x1After = depthArray2[i][j].volumePoint[0];
						double y1After = depthArray2[i][j].volumePoint[1];
						double z1After = depthArray2[i][j].volumePoint[2];

						double x2After = depthArray2[i+1][j].volumePoint[0];
						double y2After =  depthArray2[i+1][j].volumePoint[1];
						double z2After = depthArray2[i+1][j].volumePoint[2];

						double x3After = depthArray2[i][j+1].volumePoint[0];
						double y3After =  depthArray2[i][j+1].volumePoint[1];
						double z3After = depthArray2[i][j+1].volumePoint[2];

						double x4After = depthArray2[i+1][j+1].volumePoint[0];
						double y4After =  depthArray2[i+1][j+1].volumePoint[1];
						double z4After = depthArray2[i+1][j+1].volumePoint[2];

						double deltaX = ((x1Before - x2Before));// + (x3Before - x4Before) + (x1After - x2After) + (x3After - x4After))/4;
						double deltaY = ((y1Before - y3Before));// + (y2Before - y4Before) + (y1After - y3After) + (y2After - y4After))/4;

						double minZ;
						double maxZ;

						double minZBefore;
						double maxZAfter;

						minZBefore = z1Before;
						if(z2Before < minZBefore)
						{
							minZBefore = z2Before;
						}
						if(z3Before < minZBefore)
						{
							minZBefore = z3Before;
						}
						if(z4Before < minZBefore)
						{
							minZBefore = z4Before;
						}

						maxZAfter = z1After;
						if(z2After > maxZAfter)
						{
							maxZAfter = z2After;
						}
						if(z3After > maxZAfter)
						{
							maxZAfter = z3After;
						}
						if(z4After > maxZAfter)
						{
							maxZAfter = z4After;
						}

						maxZ = z1Before;
						if(z2Before > maxZ)
						{
							maxZ = z2Before;
						}
						if(z3Before > maxZ)
						{
							maxZ = z3Before;
						}
						if(z4Before > maxZ)
						{
							maxZ = z4Before;
						}

						minZ = z1After;
						if(z2After < minZ)
						{
							minZ = z2After;
						}
						if(z3After < minZ)
						{
							minZ = z3After;
						}
						if(z3After < minZ)
						{
							minZ = z3After;
						}

						double deltaZBefore = maxZ - minZBefore;
						double deltaZAfter = maxZAfter - minZ;

						printf("minZ = %f, maxZ = %f, z1b = %f, z2b = %f, z3b = %f, z4b = %f, z1a = %f, z2a = %f, z3a = %f, z4a = %f\n", minZ, maxZ, z1Before, z2Before, z3Before, z4Before, z1After, z2After, z3After, z4After);
						printf("deltaX = %f, x1B-x2B = %f, x3B-x4B = %f, x1A-x2A = %f, x3A-x4A = %f\n", deltaX, x1Before - x2Before, x3Before - x4Before, x1After - x2After, x3After - x4After);
						printf("deltaY = %f, y1B-y3B = %f, y2B-y4B = %f, y1A-y3A = %f, y2A-y4A = %f\n", deltaY, y1Before - y3Before, y2Before - y4Before, y1After - y3After, y2After - y4After);
						printf("deltaZBefore = %f, deltaZAfter = %f\n", deltaZBefore, deltaZAfter);

						double fourPointVolume = abs(((deltaX * deltaY) * ((maxZ - minZ) - ((1/2) * (deltaX * deltaY) * (deltaZBefore + deltaZAfter)))));
						//printf("volume between the 4 points = %f\n", fourPointVolume);
						volume += fourPointVolume;
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
