#include "VolumeTool.h"


#include <iostream>
#include <algorithm>
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

				// p0[0] = -538.108260;
				// p0[1] = 402.579461;
				// p1[0] = -122.329473;
				// p1[1] = 85.795623;

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

				beforeDepthArray = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					beforeDepthArray[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						beforeDepthArray[i][j] = new double [3];
					}
				}

				b1 = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					b1[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						b1[i][j] = new double [3];
					}
				}

				b2 = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					b2[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						b2[i][j] = new double [3];
					}
				}		

				b3 = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					b3[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						b3[i][j] = new double [3];
					}
				}

				b4 = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					b4[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						b4[i][j] = new double [3];
					}
				}	

				b5 = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					b5[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						b5[i][j] = new double [3];
					}
				}	

				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);

						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);

						b1[xPos][yPos][0] = worldPosition[0];
						b1[xPos][yPos][1] = worldPosition[1];
						b1[xPos][yPos][2] = worldPosition[2];
					}
				}
				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);

						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);

						b2[xPos][yPos][0] = worldPosition[0];
						b2[xPos][yPos][1] = worldPosition[1];
						b2[xPos][yPos][2] = worldPosition[2];
					}
				}
				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);

						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);

						b3[xPos][yPos][0] = worldPosition[0];
						b3[xPos][yPos][1] = worldPosition[1];
						b3[xPos][yPos][2] = worldPosition[2];
					}
				}
				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);

						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);

						b4[xPos][yPos][0] = worldPosition[0];
						b4[xPos][yPos][1] = worldPosition[1];
						b4[xPos][yPos][2] = worldPosition[2];
					}
				}
				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);

						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);

						b5[xPos][yPos][0] = worldPosition[0];
						b5[xPos][yPos][1] = worldPosition[1];
						b5[xPos][yPos][2] = worldPosition[2];
					}
				}

				std::ofstream outFile;
				outFile.open("matrix1.csv");

				std::ofstream pointFile;
				pointFile.open("points1.csv");

				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);
						
						//Code for median of 5 collections
						//double medianArray[5];
						// medianArray[0] = b1[xPos][yPos][0];
						// medianArray[1] = b2[xPos][yPos][0];
						// medianArray[2] = b3[xPos][yPos][0];
						// medianArray[3] = b4[xPos][yPos][0];
						// medianArray[4] = b5[xPos][yPos][0];

						// std::sort(medianArray, medianArray+5);
						// beforeDepthArray[xPos][yPos][0] = medianArray[2];

						// medianArray[0] = b1[xPos][yPos][1];
						// medianArray[1] = b2[xPos][yPos][1];
						// medianArray[2] = b3[xPos][yPos][1];
						// medianArray[3] = b4[xPos][yPos][1];
						// medianArray[4] = b5[xPos][yPos][1];

						// std::sort(medianArray, medianArray+5);
						// beforeDepthArray[xPos][yPos][1] = medianArray[2];

						// medianArray[0] = b1[xPos][yPos][2];
						// medianArray[1] = b2[xPos][yPos][2];
						// medianArray[2] = b3[xPos][yPos][2];
						// medianArray[3] = b4[xPos][yPos][2];
						// medianArray[4] = b5[xPos][yPos][2];

						// std::sort(medianArray, medianArray+5);
						// beforeDepthArray[xPos][yPos][2] = medianArray[2];


						// Code for mean of 5 collections
						// double mean = (b1[xPos][yPos][0] + b2[xPos][yPos][0] + b3[xPos][yPos][0] + b4[xPos][yPos][0] + b5[xPos][yPos][0])/5;
						// beforeDepthArray[xPos][yPos][0] = mean;

						// mean = (b1[xPos][yPos][1] + b2[xPos][yPos][1] + b3[xPos][yPos][1] + b4[xPos][yPos][1] + b5[xPos][yPos][1])/5;
						// beforeDepthArray[xPos][yPos][1] = mean;

						// mean = (b1[xPos][yPos][2] + b2[xPos][yPos][2] + b3[xPos][yPos][2] + b4[xPos][yPos][2] + b5[xPos][yPos][2])/5;
						// beforeDepthArray[xPos][yPos][2] = mean;
						
						// Code for no filtering of values (uses first collection)
						beforeDepthArray[xPos][yPos][0] = b1[xPos][yPos][0];
						beforeDepthArray[xPos][yPos][1] = b1[xPos][yPos][1];
						beforeDepthArray[xPos][yPos][2] = b1[xPos][yPos][2];

						pointFile <<  beforeDepthArray[xPos][yPos][0] << "," << beforeDepthArray[xPos][yPos][1] << "," << beforeDepthArray[xPos][yPos][2] << "\n";

						outFile << beforeDepthArray[xPos][yPos][2] << ",";
					}
					outFile << "\n";
				}

				outFile.close();
				pointFile.close();
			}
			else
			{

				afterDepthArray = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					afterDepthArray[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						afterDepthArray[i][j] = new double [3];
					}
				}

				a1 = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					a1[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						a1[i][j] = new double [3];
					}
				}

				a2 = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					a2[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						a2[i][j] = new double [3];
					}
				}		

				a3 = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					a3[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						a3[i][j] = new double [3];
					}
				}

				a4 = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					a4[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						a4[i][j] = new double [3];
					}
				}	

				a5 = new double **[xSize];
				for(int i = 0; i < xSize; i++)
				{
					a5[i] = new double *[ySize];
					for(int j = 0; j < ySize; j++)
					{
						a5[i][j] = new double [3];
					}
				}	

				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);

						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);

						a1[xPos][yPos][0] = worldPosition[0];
						a1[xPos][yPos][1] = worldPosition[1];
						a1[xPos][yPos][2] = worldPosition[2];
					}
				}
				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);

						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);

						a2[xPos][yPos][0] = worldPosition[0];
						a2[xPos][yPos][1] = worldPosition[1];
						a2[xPos][yPos][2] = worldPosition[2];
					}
				}
				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);

						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);

						a3[xPos][yPos][0] = worldPosition[0];
						a3[xPos][yPos][1] = worldPosition[1];
						a3[xPos][yPos][2] = worldPosition[2];
					}
				}
				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);

						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);

						a4[xPos][yPos][0] = worldPosition[0];
						a4[xPos][yPos][1] = worldPosition[1];
						a4[xPos][yPos][2] = worldPosition[2];
					}
				}
				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						Point position;

						position[0] = xValStart;
						position[1] = yValStart;

						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);

						RawKinectViewer::CPoint imagePosition = application->getDepthImagePoint(position);
						RawKinectViewer::CPoint worldPosition = application->intrinsicParameters.depthProjection.transform(imagePosition);

						a5[xPos][yPos][0] = worldPosition[0];
						a5[xPos][yPos][1] = worldPosition[1];
						a5[xPos][yPos][2] = worldPosition[2];
					}
				}

				std::ofstream outFile2;
				outFile2.open("matrix2.csv");

				std::ofstream pointFile2;
				pointFile2.open("points2.csv");

				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);

						//Code for median of 5 collections
						//double medianArray[5];
						// medianArray[0] = a1[xPos][yPos][0];
						// medianArray[1] = a2[xPos][yPos][0];
						// medianArray[2] = a3[xPos][yPos][0];
						// medianArray[3] = a4[xPos][yPos][0];
						// medianArray[4] = a5[xPos][yPos][0];

						// std::sort(medianArray, medianArray+5);
						// afterDepthArray[xPos][yPos][0] = medianArray[2];

						// medianArray[0] = a1[xPos][yPos][1];
						// medianArray[1] = a2[xPos][yPos][1];
						// medianArray[2] = a3[xPos][yPos][1];
						// medianArray[3] = a4[xPos][yPos][1];
						// medianArray[4] = a5[xPos][yPos][1];

						// std::sort(medianArray, medianArray+5);
						// afterDepthArray[xPos][yPos][1] = medianArray[2];

						// afterDepthArray[xPos][yPos][0] = beforeDepthArray[xPos][yPos][0];
						// afterDepthArray[xPos][yPos][1] = beforeDepthArray[xPos][yPos][1];

						// medianArray[0] = a1[xPos][yPos][2];
						// medianArray[1] = a2[xPos][yPos][2];
						// medianArray[2] = a3[xPos][yPos][2];
						// medianArray[3] = a4[xPos][yPos][2];
						// medianArray[4] = a5[xPos][yPos][2];

						// std::sort(medianArray, medianArray+5);
						// afterDepthArray[xPos][yPos][2] = medianArray[2];
						

						//Code for mean of 5 collections
						// double mean = (a1[xPos][yPos][0] + a2[xPos][yPos][0] + a3[xPos][yPos][0] + a4[xPos][yPos][0] + a5[xPos][yPos][0])/5;
						// afterDepthArray[xPos][yPos][0] = mean;

						// mean = (a1[xPos][yPos][1] + a2[xPos][yPos][1] + a3[xPos][yPos][1] + a4[xPos][yPos][1] + a5[xPos][yPos][1])/5;
						// afterDepthArray[xPos][yPos][1] = mean;

						// mean = (a1[xPos][yPos][2] + a2[xPos][yPos][2] + a3[xPos][yPos][2] + a4[xPos][yPos][2] + a5[xPos][yPos][2])/5;
						// afterDepthArray[xPos][yPos][2] = mean;

						// Code for no filtering of values (uses first collection)
						afterDepthArray[xPos][yPos][0] = a1[xPos][yPos][0];
						afterDepthArray[xPos][yPos][1] = a1[xPos][yPos][1];
						afterDepthArray[xPos][yPos][2] = a1[xPos][yPos][2];

						pointFile2 <<  afterDepthArray[xPos][yPos][0] << "," << afterDepthArray[xPos][yPos][1] << "," << afterDepthArray[xPos][yPos][2] << "\n";

						outFile2 << afterDepthArray[xPos][yPos][2] << ",";
					}
					outFile2 << "\n";
				}

				outFile2.close();
				pointFile2.close();

				//Volume calculation code			
				double volumePos = 0;
				double volumeNeg = 0;

				for(int i = 0; i < xSize-1; i++) //xSize
				{

					for(int j = 0; j < ySize-1; j++) //ySize
					{
						double x1Before = beforeDepthArray[i][j][0];
						double y1Before = beforeDepthArray[i][j][1];
						double z1Before = abs(beforeDepthArray[i][j][2]);

						double x2Before = beforeDepthArray[i+1][j][0];
						double y2Before =  beforeDepthArray[i+1][j][1];
						double z2Before = abs(beforeDepthArray[i+1][j][2]);

						double x3Before = beforeDepthArray[i][j+1][0];
						double y3Before =  beforeDepthArray[i][j+1][1];
						double z3Before = abs(beforeDepthArray[i][j+1][2]);

						double x4Before = beforeDepthArray[i+1][j+1][0];
						double y4Before =  beforeDepthArray[i+1][j+1][1];
						double z4Before = abs(beforeDepthArray[i+1][j+1][2]);

						double x1After = afterDepthArray[i][j][0];
						double y1After = afterDepthArray[i][j][1];
						double z1After = abs(afterDepthArray[i][j][2]);

						double x2After = afterDepthArray[i+1][j][0];
						double y2After =  afterDepthArray[i+1][j][1];
						double z2After = abs(afterDepthArray[i+1][j][2]);

						double x3After = afterDepthArray[i][j+1][0];
						double y3After =  afterDepthArray[i][j+1][1];
						double z3After = abs(afterDepthArray[i][j+1][2]);

						double x4After = afterDepthArray[i+1][j+1][0];
						double y4After =  afterDepthArray[i+1][j+1][1];
						double z4After = abs(afterDepthArray[i+1][j+1][2]);

						double deltaX = (((x1Before - x2Before)) + (x3Before - x4Before) + (x1After - x2After) + (x3After - x4After))/4;
						double deltaY = abs((((y1Before - y3Before)) + (y2Before - y4Before) + (y1After - y3After) + (y2After - y4After))/4);

						double minZ;
						double maxZ;

						double minZBefore;
						double maxZBefore;

						double minZAfter;
						double maxZAfter;

						//Minimum depth to reduce error?
						//if( (abs(z1Before - z1After) > 0.8) || (abs(z2Before - z2After) > 0.8) || (abs(z3Before - z3After) > 0.8) || (abs(z4Before - z4After) > 0.8))
						//{
						if( (z1Before != z1After) || (z2Before != z2After) || (z3Before != z3After) || (z4Before != z4After))
						{
							maxZ = z1Before;
							if(z2Before > maxZ)
							{
								maxZ = z2Before;
							}
							if(z3Before > maxZ)
							{
								maxZ = z3Before;
							}
							if(z3Before > maxZ)
							{
								maxZ = z3Before;
							}
							if(z1After > maxZ)
							{
								maxZ = z1After;
							}
							if(z2After > maxZ)
							{
								maxZ = z2After;
							}
							if(z3After > maxZ)
							{
								maxZ = z3After;
							}
							if(z4After > maxZ)
							{
								maxZ = z4After;
							}

							minZ = z1Before;
							if(z2Before < minZ)
							{
								minZ = z2Before;
							}
							if(z3Before < minZ)
							{
								minZ = z3Before;
							}
							if(z3Before < minZ)
							{
								minZ = z3Before;
							}
							if(z1After < minZ)
							{
								minZ = z1After;
							}
							if(z2After < minZ)
							{
								minZ = z2After;
							}
							if(z3After < minZ)
							{
								minZ = z3After;
							}
							if(z4After < minZ)
							{
								minZ = z4After;
							}

							maxZBefore = z1Before;
							if(z2Before > maxZBefore)
							{
								maxZBefore = z2Before;
							}
							if(z3Before > maxZBefore)
							{
								maxZBefore = z3Before;
							}
							if(z3Before > maxZBefore)
							{
								maxZBefore = z3Before;
							}

							minZBefore = z1Before;
							if(z2Before < minZBefore)
							{
								minZBefore = z2Before;
							}
							if(z3Before < minZBefore)
							{
								minZBefore = z3Before;
							}
							if(z3Before < minZBefore)
							{
								minZBefore = z3Before;
							}

							maxZAfter = z1After;
							if(z2After > maxZ)
							{
								maxZ = z2After;
							}
							if(z3After > maxZ)
							{
								maxZ = z3After;
							}
							if(z4After > maxZ)
							{
								maxZ = z4After;
							}

							minZAfter = z1After;
							if(z2After < minZAfter)
							{
								minZAfter = z2After;
							}
							if(z3After < minZAfter)
							{
								minZAfter = z3After;
							}
							if(z4After < minZAfter)
							{
								minZAfter = z4After;
							}

							double deltaZBefore = maxZBefore - minZBefore;
							double deltaZAfter = maxZAfter - minZBefore;

							double fourPointVolume = ((deltaX * deltaY) * ((maxZ - minZ) - ((1/2) * (deltaX * deltaY) * (deltaZBefore + deltaZAfter))));
							//printf("((%f * %f) * ((%f - %f) - ((1/2) * (%f * %f) * (%f + %f))))\n", deltaX, deltaY, maxZ, minZ, deltaX, deltaY, deltaZBefore, deltaZAfter);
							//printf("volume between the 4 points = %f\n", fourPointVolume);
							if(fourPointVolume < 0){
								volumeNeg += fourPointVolume;
							} else {
								volumePos += fourPointVolume;
							}
						}
						
					}
				}
				printf("volumePos = %f, volumeNeg = %f, volumeNet = %f\n", volumePos, volumeNeg, (volumeNeg + volumePos));
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
