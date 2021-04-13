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

				//If z value is an unexpected extreme, average over surrounding values
				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);
						if ((beforeDepthArray[xPos][yPos][2] > -110) || (beforeDepthArray[xPos][yPos][2] < -200))
						{
							//printf("z is extreme = %f\n", beforeDepthArray[xPos][yPos][2] );
							if((xPos > 0) && (yPos > 0))
							{
								double average = (beforeDepthArray[xPos-1][yPos-1][2] + beforeDepthArray[xPos-1][yPos][2] + 
									beforeDepthArray[xPos-1][yPos+1][2] + beforeDepthArray[xPos][yPos-1][2] + beforeDepthArray[xPos][yPos+1][2] + 
									beforeDepthArray[xPos+1][yPos-1][2] + beforeDepthArray[xPos+1][yPos][2] + beforeDepthArray[xPos+1][yPos+1][2])/8;

								beforeDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == 0) && (yPos == 0))
							{
								double average = (beforeDepthArray[xPos+1][yPos][2] + beforeDepthArray[xPos][yPos+1][2] + 
									beforeDepthArray[xPos+1][yPos+1][2])/3;

								beforeDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == 0) && (yPos == ySize-1))
							{
								double average = (beforeDepthArray[xPos][yPos-1][2] + beforeDepthArray[xPos+1][yPos-1][2] + 
									beforeDepthArray[xPos+1][yPos][2])/3;

								beforeDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == xSize-1) && (yPos == 0))
							{
								double average = (beforeDepthArray[xPos-1][yPos][2] + beforeDepthArray[xPos-1][yPos-1][2] + 
									beforeDepthArray[xPos][yPos+1][2])/3;

								beforeDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == xSize-1) && (yPos == ySize-1))
							{
								double average = (beforeDepthArray[xPos-1][yPos][2] + beforeDepthArray[xPos-1][yPos-1][2] + 
									beforeDepthArray[xPos][yPos-1][2])/3;

								beforeDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == 0))
							{
								double average = (beforeDepthArray[xPos][yPos-1][2] + beforeDepthArray[xPos+1][yPos-1][2] + 
									beforeDepthArray[xPos+1][yPos][2] + beforeDepthArray[xPos+1][yPos+1][2] + beforeDepthArray[xPos][yPos+1][2])/5;

								beforeDepthArray[xPos][yPos][2] = average;
							}
							else if((yPos == 0))
							{
								double average = (beforeDepthArray[xPos-1][yPos][2] + beforeDepthArray[xPos-1][yPos+1][2] + 
									beforeDepthArray[xPos][yPos+1][2] + beforeDepthArray[xPos+1][yPos+1][2] + beforeDepthArray[xPos+1][yPos][2])/5;

								beforeDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == xSize-1))
							{
								double average = (beforeDepthArray[xPos][yPos-1][2] + beforeDepthArray[xPos-1][yPos-1][2] + 
									beforeDepthArray[xPos-1][yPos][2] + beforeDepthArray[xPos-1][yPos+1][2] + beforeDepthArray[xPos][yPos+1][2])/5;

								beforeDepthArray[xPos][yPos][2] = average;
							}
							else if((yPos == ySize-1))
							{
								double average = (beforeDepthArray[xPos-1][yPos][2] + beforeDepthArray[xPos-1][yPos-1][2] + 
									beforeDepthArray[xPos][yPos-1][2] + beforeDepthArray[xPos+1][yPos-1][2] + beforeDepthArray[xPos+1][yPos][2])/5;

								beforeDepthArray[xPos][yPos][2] = average;
							}
						}
					}
				}

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

				//If z value is an unexpected extreme, average over surrounding values
				for(double xValStart = floor(p0[0]); xValStart < xValEnd; xValStart = xValStart + 1)
				{
					for(double yValStart = floor(p0[1]); yValStart > yValEnd; yValStart = yValStart - 1)
					{
						int xPos = ((xValEnd - xValStart) -1);
						int yPos = ((yValStart - yValEnd) -1);
						if ((afterDepthArray[xPos][yPos][2] > -110) || (afterDepthArray[xPos][yPos][2] < -200))
						{
							//printf("z is extreme = %f\n", afterDepthArray[xPos][yPos][2] );
							if((xPos > 0) && (yPos > 0))
							{
								double average = (afterDepthArray[xPos-1][yPos-1][2] + afterDepthArray[xPos-1][yPos][2] + 
									afterDepthArray[xPos-1][yPos+1][2] + afterDepthArray[xPos][yPos-1][2] + afterDepthArray[xPos][yPos+1][2] + 
									afterDepthArray[xPos+1][yPos-1][2] + afterDepthArray[xPos+1][yPos][2] + afterDepthArray[xPos+1][yPos+1][2])/8;

								afterDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == 0) && (yPos == 0))
							{
								double average = (afterDepthArray[xPos+1][yPos][2] + afterDepthArray[xPos][yPos+1][2] + 
									afterDepthArray[xPos+1][yPos+1][2])/3;

								afterDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == 0) && (yPos == ySize-1))
							{
								double average = (afterDepthArray[xPos][yPos-1][2] + afterDepthArray[xPos+1][yPos-1][2] + 
									afterDepthArray[xPos+1][yPos][2])/3;

								afterDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == xSize-1) && (yPos == 0))
							{
								double average = (afterDepthArray[xPos-1][yPos][2] + afterDepthArray[xPos-1][yPos-1][2] + 
									afterDepthArray[xPos][yPos+1][2])/3;

								afterDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == xSize-1) && (yPos == ySize-1))
							{
								double average = (afterDepthArray[xPos-1][yPos][2] + afterDepthArray[xPos-1][yPos-1][2] + 
									afterDepthArray[xPos][yPos-1][2])/3;

								afterDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == 0))
							{
								double average = (afterDepthArray[xPos][yPos-1][2] + afterDepthArray[xPos+1][yPos-1][2] + 
									afterDepthArray[xPos+1][yPos][2] + afterDepthArray[xPos+1][yPos+1][2] + afterDepthArray[xPos][yPos+1][2])/5;

								afterDepthArray[xPos][yPos][2] = average;
							}
							else if((yPos == 0))
							{
								double average = (afterDepthArray[xPos-1][yPos][2] + afterDepthArray[xPos-1][yPos+1][2] + 
									afterDepthArray[xPos][yPos+1][2] + afterDepthArray[xPos+1][yPos+1][2] + afterDepthArray[xPos+1][yPos][2])/5;

								afterDepthArray[xPos][yPos][2] = average;
							}
							else if((xPos == xSize-1))
							{
								double average = (afterDepthArray[xPos][yPos-1][2] + afterDepthArray[xPos-1][yPos-1][2] + 
									afterDepthArray[xPos-1][yPos][2] + afterDepthArray[xPos-1][yPos+1][2] + afterDepthArray[xPos][yPos+1][2])/5;

								afterDepthArray[xPos][yPos][2] = average;
							}
							else if((yPos == ySize-1))
							{
								double average = (afterDepthArray[xPos-1][yPos][2] + afterDepthArray[xPos-1][yPos-1][2] + 
									afterDepthArray[xPos][yPos-1][2] + afterDepthArray[xPos+1][yPos-1][2] + afterDepthArray[xPos+1][yPos][2])/5;

								afterDepthArray[xPos][yPos][2] = average;
							}
						}
					}
				}

				//Volume calculation code			
				double volumePos = 0;
				double volumeNeg = 0;
				double totalError = 0;
				double negativeError = 0;
				double positiveError = 0;
				std::vector<double> errorList;
				std::vector<double> negativeErrorList;
				std::vector<double> positiveErrorList;

				std::ofstream volumePointFile;
				volumePointFile.open("volumePointFile.csv");

				//Test 5x5 matrix for sanity
				// double ***newBefore;

				// newBefore = new double **[5];
				// for(int i = 0; i < 5; i++)
				// {
				// 	newBefore[i] = new double *[5];
				// 	for(int j = 0; j < 5; j++)
				// 	{
				// 		newBefore[i][j] = new double [3];
				// 	}
				// }

				// double ***newAfter;

				// newAfter = new double **[5];
				// for(int i = 0; i < 5; i++)
				// {
				// 	newAfter[i] = new double *[5];
				// 	for(int j = 0; j < 5; j++)
				// 	{
				// 		newAfter[i][j] = new double [3];
				// 	}
				// }

				// double x = 0;
				// double y = 0;
				// double z = 0;

				// for(int i = 0; i < 5; i++)
				// {
				// 	y = 0;
				// 	for(int j = 0; j < 5; j++)
				// 	{
				// 		newBefore[i][j][0] = x;
				// 		newBefore[i][j][1] = y;
				// 		newBefore[i][j][2] = z;

				// 		newAfter[i][j][0] = x;
				// 		newAfter[i][j][1] = y;
				// 		newAfter[i][j][2] = z;
				// 		y += 0.30;
				// 	}
				// 	x += 0.30;
				// }

				// newAfter[1][1][2] = 10.0;
				// newAfter[1][2][2]= 10.0;
				// newAfter[1][3][2] = 10.0;

				// newAfter[2][1][2] = 10.0;
				// newAfter[2][2][2] = 10.0;
				// newAfter[2][3][2] = 10.0;

				// newAfter[3][1][2] = 10.0;
				// newAfter[3][2][2] = 10.0;
				// newAfter[3][3][2] = 10.0;

				//Initial average for handling extreme x and y dimensions
				double xDimAverage = 0; //0.27;
				double yDimAverage = 0; //0.27;

				double averageZ = 0;

				//double calcCount = 0;
				//double weirdCaseCount = 0;

				// double xError = 0;
				// int xCount = 0;
				// double yError = 0;
				// int yCount = 0;
				// double zError = 0;
				// int zCount = 0;

				double singleX = 0;
				double singleY = 0;

				for(int i = 0; i < xSize-1; i++) //xSize
				{
					double x1Before = beforeDepthArray[i][0][0];
					double x2Before = beforeDepthArray[i+1][0][0];

					singleX += abs(x1Before - x2Before);
				}
				//printf("x length = %f\n", singleX);

				for(int i = 0; i < ySize-1; i++) //xSize
				{
					double y1Before = beforeDepthArray[0][i][1];
					double y3Before = beforeDepthArray[0][i+1][1];
					singleY += abs(y3Before - y1Before);
				}
				//printf("y length = %f\n", singleY);
				singleY = 0;


				for(int i = 0; i < xSize-1; i++) //xSize
				{
					for(int j = 0; j < ySize-1; j++) //ySize
					{
						//Set the 4 before and 4 after points
						double x1Before = beforeDepthArray[i][j][0];
						double y1Before = beforeDepthArray[i][j][1];
						double z1Before = beforeDepthArray[i][j][2];

						double x2Before = beforeDepthArray[i+1][j][0];
						double y2Before = beforeDepthArray[i+1][j][1];
						double z2Before = beforeDepthArray[i+1][j][2];;

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

						//Points to use for the test matrix, need to set the for loops to 5-1 as well
						// double x1Before = newAfter[i][j][0];
						// double y1Before = newAfter[i][j][1];
						// double z1Before = newAfter[i][j][2];

						// double x2Before = newAfter[i+1][j][0];
						// double y2Before = newAfter[i+1][j][1];
						// double z2Before = newAfter[i+1][j][2];

						// double x3Before = newAfter[i][j+1][0];
						// double y3Before = newAfter[i][j+1][1];
						// double z3Before = newAfter[i][j+1][2];

						// double x4Before = newAfter[i+1][j+1][0];
						// double y4Before = newAfter[i+1][j+1][1];
						// double z4Before = newAfter[i+1][j+1][2];

						// double x1After = newBefore[i][j][0];
						// double y1After = newBefore[i][j][1];
						// double z1After = newBefore[i][j][2];

						// double x2After = newBefore[i+1][j][0];
						// double y2After = newBefore[i+1][j][1];
						// double z2After = newBefore[i+1][j][2];

						// double x3After = newBefore[i][j+1][0];
						// double y3After = newBefore[i][j+1][1];
						// double z3After = newBefore[i][j+1][2];

						// double x4After = newBefore[i+1][j+1][0];
						// double y4After = newBefore[i+1][j+1][1];
						// double z4After = newBefore[i+1][j+1][2];


						// if(abs(x1Before-x1After) > 0){
						// 	xCount++;
						// 	xError += abs(x1Before-x1After);
						// }
						// if(abs(x2Before-x2After) > 0){
						// 	xCount++;
						// 	xError += abs(x2Before-x2After);
						// }
						// if(abs(x3Before-x3After) > 0){
						// 	xCount++;
						// 	xError += abs(x3Before-x3After);
						// }
						// if(abs(x4Before-x4After) > 0){
						// 	xCount++;
						// 	xError += abs(x4Before-x4After);
						// }
						
						// if(abs(y1Before-y1After) > 0){
						// 	yCount++;
						// 	yError += abs(y1Before-y1After);
						// }
						// if(abs(y2Before-y2After) > 0){
						// 	yCount++;
						// 	yError += abs(y2Before-y2After);
						// }
						// if(abs(y3Before-y3After) > 0){
						// 	yCount++;
						// 	yError += abs(y3Before-y3After);
						// }
						// if(abs(y4Before-y4After) > 0){
						// 	yCount++;
						// 	yError += abs(y4Before-y4After);
						// }

						// if(abs(z1Before-z1After) > 0){
						// 	zCount++;
						// 	zError += abs(z1Before-z1After);
						// }
						// if(abs(z2Before-z2After) > 0){
						// 	zCount++;
						// 	zError += abs(z2Before-z2After);
						// }
						// if(abs(z3Before-z3After) > 0){
						// 	zCount++;
						// 	zError += abs(z3Before-z3After);
						// }
						// if(abs(z4Before-z4After) > 0){
						// 	zCount++;
						// 	zError += abs(z4Before-z4After);
						// }


						//Substract the expected error at the given z value for this dimension
						double deltaX1 = abs(x1Before - x2Before) - ((abs(x1Before - x2Before) * (-0.0022*((z1Before+z2Before)/2) - 0.2976)));
						double deltaX2 = abs(x3Before - x4Before) - ((abs(x3Before - x4Before) * (-0.0022*((z3Before+z4Before)/2) - 0.2976)));
						double deltaX3 = abs(x1After - x2After) - ((abs(x1After - x2After) * (-0.0022*((z1After+z2After)/2) - 0.2976)));
						double deltaX4 = abs(x3After - x4After) - ((abs(x3After - x4After) * (-0.0022*((z3After+z4After)/2) - 0.2976)));

						double xDim = (deltaX1+deltaX2+deltaX3+deltaX4)/4;

						// double xDim = (abs(x1Before - x2Before) + abs(x3Before - x4Before) + abs(x1After - x2After) + abs(x3After - x4After))/4;
						
						//printf("xDim = %f\n", xDim);

						double deltaY1 = abs(y3Before - y1Before) - (abs(y3Before - y1Before) * (-0.0028*((z3Before+z1Before)/2) - 0.3906));
						double deltaY2 = abs(y4Before - y2Before) - (abs(y4Before - y2Before) * (-0.0028*((z4Before+z2Before)/2) - 0.3906));
						double deltaY3 = abs(y3After - y1After) - (abs(y3After - y1After) * (-0.0028*((z3After+z1After)/2) - 0.3906));
						double deltaY4 = abs(y4After - y2After) - (abs(y4After - y2After) * (-0.0028*((z4After+z2After)/2) - 0.3906));

						double yDim = (deltaY1+deltaY2+deltaY3+deltaY4)/4;

						// double yDim = (abs(y3Before - y1Before) + abs(y4Before - y2Before) + abs(y3After - y1After) + abs(y4After - y2After))/4;
						
						//printf("yDim = %f\n", yDim);

						if((xDimAverage == 0) && (yDimAverage == 0))
						{
							xDimAverage = xDim;
							yDimAverage = yDim;
						}
						
						//x dimension is more than 2 times, set it to the average
						if((xDim > (xDimAverage*2)) || (xDim < (xDimAverage/2)))
						{
							xDim = xDimAverage;
						}
						//add to the average
						xDimAverage = (xDimAverage + xDim)/2;

						//y dimension is more than 2 times, set it to the average
						if((yDim > (yDimAverage*2)) || (yDim < (yDimAverage/2)))
						{
							yDim = yDimAverage;
						}
						//add to the average
						yDimAverage = (yDimAverage + yDim)/2;


						averageZ = (z1Before + z2Before + z3Before + z4Before + z1After + z2After + z3After + z4After)/8;

						double minZ = 0;
						double maxZ = 0;
						bool conflictingPoints = false;

						double minZBefore;
						double maxZBefore;

						double minZAfter;
						double maxZAfter;

						//Minimum depth change to reduce error
						double minDepth = 0.9;
						if( (abs(z1Before - z1After) > minDepth) && (abs(z2Before - z2After) > minDepth) && (abs(z3Before - z3After) > minDepth) && (abs(z4Before - z4After) > minDepth))
						{		
							singleY += yDim;

							maxZBefore = std::max({z1Before, z2Before, z3Before, z4Before});
							minZBefore = std::min({z1Before, z2Before, z3Before, z4Before});
							maxZAfter = std::max({z1After, z2After, z3After, z4After});
							minZAfter = std::min({z1After, z2After, z3After, z4After});

							if((z1Before > z1After) && (z2Before > z2After) && (z3Before > z3After) && (z4Before > z4After))
							{
								maxZ = maxZBefore;
								minZ = minZAfter;
							}
							else if((z1Before < z1After) && (z2Before < z2After) && (z3Before < z3After) && (z4Before < z4After))
							{
								maxZ = minZBefore;
								minZ = maxZAfter;
							}
							else
							{
								// This case occurs very rarely when the before points are not all greater
								// or less than the after. Volume of these cases is ignored since it is 
								// minimal. 10-25 times out of 21000
								conflictingPoints = true;
								//weirdCaseCount++;
							}
							
							if(!conflictingPoints)
							{
								double deltaZBefore = maxZBefore - minZBefore;
								double deltaZAfter = maxZAfter - minZAfter;

								double xDimError = 0.12;
								double yDimError = 0.10; 
								double zDimError = 0.90;

								double errorXDim = sqrt(pow(xDimError, 2) * 8) * 0.25;
								double errorYDim = sqrt(pow(yDimError, 2) * 8) * 0.25;

								double errorDeltaZs = sqrt(pow(zDimError, 2) * 4);
								double errorZs = sqrt(pow(zDimError, 2) * 2);

								double volumePart1 = (0.5) * (deltaZBefore + deltaZAfter);

								double errorPart1 = errorDeltaZs/2;

								double volumePart2;
								if((maxZ-minZ) > 0){
									volumePart2 = ((maxZ - minZ) - volumePart1);
								}
								else
								{
									volumePart2 = ((maxZ - minZ) + volumePart1);
								}
									
								double errorPart2 = sqrt(pow(errorZs, 2) + pow(errorPart1, 2));

								double volumeCalculation = ((xDim * yDim) * volumePart2);
								//printf("x = %f, y = %f, z = %f\n", xDim, yDim, volumePart2);

								double volumeError;
								if (volumePart2 != 0)
								{
									volumeError = abs(volumeCalculation) * sqrt(pow((errorXDim/xDim), 2) + pow((errorYDim/yDim), 2) +  pow((errorPart2/volumePart2), 2));
								}
								else
								{
									volumeError = abs(volumeCalculation) * sqrt(pow((errorXDim/xDim), 2) + pow((errorYDim/yDim), 2));
								}
								
								errorList.emplace_back(volumeError);

								volumePointFile <<  x1Before << ", " << y1Before << ", " << z1Before << ", " << x2Before << ", " << y2Before << ", " << z2Before << "\n";
								volumePointFile <<  x3Before << ", " << y3Before << ", " << z3Before << ", " << x4Before << ", " << y4Before << ", " << z4Before << "\n\n";

								volumePointFile <<  x1After << ", " << y1After << ", " << z1After << ", " << x2After << ", " << y2After << ", " << z2After << "\n";
								volumePointFile <<  x3After << ", " << y3After << ", " << z3After << ", " << x4After << ", " << y4After << ", " << z4After << "\n\n";

								if(volumeCalculation < 0){
									volumeNeg += volumeCalculation;
									negativeErrorList.emplace_back(volumeError);
								} else {
									volumePos += volumeCalculation;
									positiveErrorList.emplace_back(volumeError);
								}
							}
						}
					}
					// printf("y length calced = %f\n", singleY);
					// singleY = 0;
				}


				volumePointFile.close();

				int errorListSize = errorList.size();
				for(int i = 0; i < errorListSize; i++)
				{
					totalError += pow(errorList[i], 2);
				}
				totalError = sqrt(totalError);

				errorListSize = negativeErrorList.size();
				for(int i = 0; i < errorListSize; i++)
				{
					negativeError += pow(negativeErrorList[i], 2);
				}
				negativeError = sqrt(negativeError);

				errorListSize = positiveErrorList.size();
				for(int i = 0; i < errorListSize; i++)
				{
					positiveError += pow(positiveErrorList[i], 2);
				}
				positiveError = sqrt(positiveError);

				//double percentWeird = weirdCaseCount/calcCount;

				printf("Average x = %f, y = %f, z = %f\n", xDimAverage, yDimAverage, averageZ);
				printf("volumeRemoved = %f +/- %f, volumeAdded = %f +/- %f, volumeNet = %f +/- %f\n", volumePos, positiveError, abs(volumeNeg), negativeError, (volumeNeg + volumePos), totalError);
				//printf("Percentage of weird case = %f/%f = %f\n", weirdCaseCount, calcCount, percentWeird);
				//printf("X Error = %f, Y Error = %f, Z Error = %f\n", xError/xCount,  yError/yCount, zError/zCount);
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
		glColor3f(0.0f,0.0f,0.0f);
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
