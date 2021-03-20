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
#include <vector>

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

				printf("top left corner:         imagePoint0 = %f, %f, %f\n", imagePoint0[0], imagePoint0[1], imagePoint0[2]);
				printf("bottom right corner:     imagePoint0 = %f, %f, %f\n", imagePoint1[0], imagePoint1[1], imagePoint1[2]);

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

				std::ofstream xMatrix;
				xMatrix.open("xMatrix.csv");

				std::ofstream pointFile;
				pointFile.open("points1.csv");

				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);
						
						//Code for median of 5 collections
						double medianArray[5];
						medianArray[0] = b1[xPos][yPos][0];
						medianArray[1] = b2[xPos][yPos][0];
						medianArray[2] = b3[xPos][yPos][0];
						medianArray[3] = b4[xPos][yPos][0];
						medianArray[4] = b5[xPos][yPos][0];

						std::sort(medianArray, medianArray+5);
						beforeDepthArray[xPos][yPos][0] = medianArray[2];

						medianArray[0] = b1[xPos][yPos][1];
						medianArray[1] = b2[xPos][yPos][1];
						medianArray[2] = b3[xPos][yPos][1];
						medianArray[3] = b4[xPos][yPos][1];
						medianArray[4] = b5[xPos][yPos][1];

						std::sort(medianArray, medianArray+5);
						beforeDepthArray[xPos][yPos][1] = medianArray[2];

						medianArray[0] = b1[xPos][yPos][2];
						medianArray[1] = b2[xPos][yPos][2];
						medianArray[2] = b3[xPos][yPos][2];
						medianArray[3] = b4[xPos][yPos][2];
						medianArray[4] = b5[xPos][yPos][2];

						std::sort(medianArray, medianArray+5);
						beforeDepthArray[xPos][yPos][2] = medianArray[2];


						// Code for mean of 5 collections
						// double mean = (b1[xPos][yPos][0] + b2[xPos][yPos][0] + b3[xPos][yPos][0] + b4[xPos][yPos][0] + b5[xPos][yPos][0])/5;
						// beforeDepthArray[xPos][yPos][0] = mean;

						// mean = (b1[xPos][yPos][1] + b2[xPos][yPos][1] + b3[xPos][yPos][1] + b4[xPos][yPos][1] + b5[xPos][yPos][1])/5;
						// beforeDepthArray[xPos][yPos][1] = mean;

						// mean = (b1[xPos][yPos][2] + b2[xPos][yPos][2] + b3[xPos][yPos][2] + b4[xPos][yPos][2] + b5[xPos][yPos][2])/5;
						// beforeDepthArray[xPos][yPos][2] = mean;
						
						// Code for no filtering of values (uses first collection)
						// beforeDepthArray[xPos][yPos][0] = b1[xPos][yPos][0];
						// beforeDepthArray[xPos][yPos][1] = b1[xPos][yPos][1];
						// beforeDepthArray[xPos][yPos][2] = b1[xPos][yPos][2];

						pointFile <<  beforeDepthArray[xPos][yPos][0] << "," << beforeDepthArray[xPos][yPos][1] << "," << beforeDepthArray[xPos][yPos][2] << "\n";

						outFile << beforeDepthArray[xPos][yPos][2] << ",";
						xMatrix << beforeDepthArray[xPos][yPos][0] << ",";
					}
					outFile << "\n";
					xMatrix << "\n";
				}

				outFile.close();
				pointFile.close();
				xMatrix.close();
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
						double medianArray[5];
						medianArray[0] = a1[xPos][yPos][0];
						medianArray[1] = a2[xPos][yPos][0];
						medianArray[2] = a3[xPos][yPos][0];
						medianArray[3] = a4[xPos][yPos][0];
						medianArray[4] = a5[xPos][yPos][0];

						std::sort(medianArray, medianArray+5);
						afterDepthArray[xPos][yPos][0] = medianArray[2];

						medianArray[0] = a1[xPos][yPos][1];
						medianArray[1] = a2[xPos][yPos][1];
						medianArray[2] = a3[xPos][yPos][1];
						medianArray[3] = a4[xPos][yPos][1];
						medianArray[4] = a5[xPos][yPos][1];

						std::sort(medianArray, medianArray+5);
						afterDepthArray[xPos][yPos][1] = medianArray[2];

						afterDepthArray[xPos][yPos][0] = beforeDepthArray[xPos][yPos][0];
						afterDepthArray[xPos][yPos][1] = beforeDepthArray[xPos][yPos][1];

						medianArray[0] = a1[xPos][yPos][2];
						medianArray[1] = a2[xPos][yPos][2];
						medianArray[2] = a3[xPos][yPos][2];
						medianArray[3] = a4[xPos][yPos][2];
						medianArray[4] = a5[xPos][yPos][2];

						std::sort(medianArray, medianArray+5);
						afterDepthArray[xPos][yPos][2] = medianArray[2];
						

						//Code for mean of 5 collections
						// double mean = (a1[xPos][yPos][0] + a2[xPos][yPos][0] + a3[xPos][yPos][0] + a4[xPos][yPos][0] + a5[xPos][yPos][0])/5;
						// afterDepthArray[xPos][yPos][0] = mean;

						// mean = (a1[xPos][yPos][1] + a2[xPos][yPos][1] + a3[xPos][yPos][1] + a4[xPos][yPos][1] + a5[xPos][yPos][1])/5;
						// afterDepthArray[xPos][yPos][1] = mean;

						// mean = (a1[xPos][yPos][2] + a2[xPos][yPos][2] + a3[xPos][yPos][2] + a4[xPos][yPos][2] + a5[xPos][yPos][2])/5;
						// afterDepthArray[xPos][yPos][2] = mean;

						// Code for no filtering of values (uses first collection)
						// afterDepthArray[xPos][yPos][0] = a1[xPos][yPos][0];
						// afterDepthArray[xPos][yPos][1] = a1[xPos][yPos][1];
						// afterDepthArray[xPos][yPos][2] = a1[xPos][yPos][2];

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
				double totalError = 0;
				std::vector<double> errorList;

				std::ofstream volumePointFile;
				volumePointFile.open("volumePointFile.csv");

				for(int i = 0; i < xSize-1; i++) //xSize
				{
					for(int j = 0; j < ySize-1; j++) //ySize
					{
						double x1Before = beforeDepthArray[i][j][0];
						double y1Before = beforeDepthArray[i][j][1];
						double z1Before = beforeDepthArray[i][j][2];

						double x2Before = beforeDepthArray[i+1][j][0];
						double y2Before = beforeDepthArray[i+1][j][1];
						double z2Before = beforeDepthArray[i+1][j][2];

						double x3Before = beforeDepthArray[i][j+1][0];
						double y3Before = beforeDepthArray[i][j+1][1];
						double z3Before = beforeDepthArray[i][j+1][2];

						double x4Before = beforeDepthArray[i+1][j+1][0];
						double y4Before = beforeDepthArray[i+1][j+1][1];
						double z4Before = beforeDepthArray[i+1][j+1][2];

						double x1After = afterDepthArray[i][j][0];
						double y1After = afterDepthArray[i][j][1];
						double z1After = afterDepthArray[i][j][2];

						double x2After = afterDepthArray[i+1][j][0];
						double y2After = afterDepthArray[i+1][j][1];
						double z2After = afterDepthArray[i+1][j][2];

						double x3After = afterDepthArray[i][j+1][0];
						double y3After = afterDepthArray[i][j+1][1];
						double z3After = afterDepthArray[i][j+1][2];

						double x4After = afterDepthArray[i+1][j+1][0];
						double y4After = afterDepthArray[i+1][j+1][1];
						double z4After = afterDepthArray[i+1][j+1][2];

						double xDim = ( abs(x1Before - x2Before) + abs(x3Before - x4Before) + abs(x1After - x2After) + abs(x3After - x4After) )/4;
						double yDim = ( abs(y3Before - y1Before) + abs(y4Before - y2Before) + abs(y3After - y1After) + abs(y4After - y2After) )/4;

						double minZ;
						double maxZ;

						double minZBefore;
						double maxZBefore;

						double minZAfter;
						double maxZAfter;

						//Minimum depth to reduce error?
						double minDepth = 0.9;
						if( (abs(z1Before - z1After) > minDepth) || (abs(z2Before - z2After) > minDepth) || (abs(z3Before - z3After) > minDepth) || (abs(z4Before - z4After) > minDepth))
						{
						//if( (z1Before != z1After) || (z2Before != z2After) || (z3Before != z3After) || (z4Before != z4After))
						//{}

							maxZ = std::max({z1Before, z2Before, z3Before, z4Before});

							minZ = std::min({z1After, z2After, z3After, z4After});
							
							maxZBefore = maxZ;

							minZBefore = std::min({z1Before, z2Before, z3Before, z4Before});

							maxZAfter = std::max({z1After, z2After, z3After, z4After});

							minZAfter = minZ;

							double deltaZBefore = maxZBefore - minZBefore;
							double deltaZAfter = maxZAfter - minZAfter;

							// printf("before min/max = %f, %f\n", minZBefore, maxZBefore);
							// printf("after min/max = %f, %f\n", minZAfter, maxZAfter);
							// printf("overall min/max = %f, %f\n", minZ, maxZ);

							double errorxDim = (1 / sqrt(4)) * sqrt(pow(0.38, 2) * 8); //???
							double erroryDim = (1 / sqrt(4)) * sqrt(pow(0.38, 2) * 8); //???

							double errorDeltaZBefore = sqrt(pow(0.9,2)*2);
							double errorDeltaZAfter = sqrt(pow(0.9,2)*2);

							double errorDeltaZs = sqrt(pow(errorDeltaZBefore, 2) + pow(errorDeltaZAfter, 2));
							double errorZs = sqrt(pow(0.9,2)*2);

							double volumePart1 = ((1/2) * (xDim * yDim) * (deltaZBefore + deltaZAfter));

							double errorPart1;
							if (volumePart1 == 0)
							{
								errorPart1 = 0;
							}
							else 
							{
								errorPart1 = volumePart1 * (1/sqrt(2)) * sqrt( pow((errorxDim/xDim), 2) + pow((erroryDim/yDim), 2) + pow(((errorDeltaZs)/(deltaZBefore+deltaZAfter)), 2)); //???
							}

							//printf("errorPart1 = %f\n", errorPart1);
							
							double volumePart2 = ((maxZ - minZ) - volumePart1);
							double errorPart2 = volumePart2 * sqrt(pow(errorZs, 2) + pow(errorPart1, 2));
							
							// printf("z1b = %f, z2b = %f, z3b = %f, z4b = %f\n", z1Before, z2Before, z3Before, z4Before);
							// printf("z1a = %f, z2a = %f, z3a = %f, z4a = %f\n", z1After, z2After, z3After, z4After);
							// printf("errorZs = %f, volumePart2 = %f, maxZ = %f, minZ = %f\n", errorZs, volumePart2, maxZ, minZ);
							// printf("errorPart2 = %f\n", errorPart2);	


							double volumeCalculation = ((xDim * yDim) * volumePart2);

							double volumeError = volumeCalculation * sqrt(pow((errorxDim/xDim), 2) + pow((erroryDim/yDim), 2) +  pow((errorPart2/volumePart2), 2) );
					
							errorList.emplace_back(volumeError);

							//printf("volume error = %f\n", volumeError);
							
							//printf("((%f * %f) * ((%f - %f) - ((1/2) * (%f * %f) * (%f + %f))))\n", xDim, yDim, maxZ, minZ, xDim, yDim, deltaZBefore, deltaZAfter);
							//printf("volume between the 4 points = %f\n", volumeCalculation);

							volumePointFile <<  x1Before << ", " << y1Before << ", " << z1Before << ", " << x2Before << ", " << y2Before << ", " << z2Before << "\n";
							volumePointFile <<  x3Before << ", " << y3Before << ", " << z3Before << ", " << x4Before << ", " << y4Before << ", " << z4Before << "\n\n";

							volumePointFile <<  x1After << ", " << y1After << ", " << z1After << ", " << x2After << ", " << y2After << ", " << z2After << "\n";
							volumePointFile <<  x3After << ", " << y3After << ", " << z3After << ", " << x4After << ", " << y4After << ", " << z4After << "\n\n";

							if(volumeCalculation < 0){
								volumeNeg += volumeCalculation;
								// volumePointFile << "above 4 lines give negative volume\n\n";
								// printf("x1b = %f, y1b = %f, z1b = %f		x2b = %f, y2b = %f, z2b = %f\n", x1Before, y1Before, z1Before, x2Before, y2Before, z2Before);
								// printf("x3b = %f, y3b = %f, z3b = %f		x4b = %f, y4b = %f, z4b = %f\n", x3Before, y3Before, z3Before, x4Before, y4Before, z4Before);

								// printf("x1a = %f, y1a = %f, z1a = %f		x2a = %f, y2a = %f, z2a = %f\n", x1After, y1After, z1After, x2After, y2After, z2After);
								// printf("x3a = %f, y3a = %f, z3a = %f		x4a = %f, y4a = %f, z4a = %f\n\n", x3After, y3After, z3After, x4After, y4After, z4After);
								
								// printf("x1b = %f, x2b = %f, x3b = %f, x4b = %f, x1a = %f, x2a = %f, x3a = %f, x4a = %f\n",x1Before, x2Before, x3Before, x4Before, x1After, x2After, x3After, x4After);
								// printf("((xDim * yDim) * ((maxZ - minZ) - ((1/2) * (xDim * yDim) * (deltaZBefore + deltaZAfter))))\n");
								// printf("((%f * %f) * ((%f - %f) - ((1/2) * (%f * %f) * (%f + %f))))\n", xDim, yDim, maxZ, minZ, xDim, yDim, deltaZBefore, deltaZAfter);
								// printf("Above = %f\n\n", volumeCalculation);
							} else {
								volumePos += volumeCalculation;
							}
						}
						
					}
				}
				volumePointFile.close();

				int errorListSize = errorList.size();
				for(int i = 0; i < errorListSize; i++)
				{
					totalError += pow(errorList[i], 2);
				}
				totalError = sqrt(totalError);

				printf("volumeRemoved = %f, volumeAdded = %f, volumeNet = %f, error = %f\n", volumePos, abs(volumeNeg), (volumeNeg + volumePos), totalError);
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
