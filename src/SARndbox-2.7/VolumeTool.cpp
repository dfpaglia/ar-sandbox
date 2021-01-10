#include "FrameFilter.h"
#include "VolumeTool.h"

#include <Vrui/ToolManager.h>

#include "Sandbox.h"

/****************************************
Static elements of class VolumeTool:
****************************************/

VolumeToolFactory* VolumeTool::factory=0;

/********************************
Methods of class VolumeTool:
********************************/
void Sandbox::receiveFilteredFrame(const Kinect::FrameBuffer& frameBuffer)
	{
	/* Put the new frame into the frame input buffer: */
	filteredFrames.postNewValue(frameBuffer);
	
	/* Wake up the foreground thread: */
	Vrui::requestUpdate();
	}

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
            std::cout << "Button pressed" << std::endl;

		if(buttonSlotIndex==0){
			std::cout << "Volume start" << std::endl;

			/*
			Math::Interval<double> elevationRange=cfg.retrieveValue<Math::Interval<double> >("./elevationRange",Math::Interval<double>(-1000.0,1000.0));
			bool haveHeightMapPlane=cfg.hasTag("./heightMapPlane");
			unsigned int numAveragingSlots=cfg.retrieveValue<unsigned int>("./numAveragingSlots",30);
			unsigned int minNumSamples=cfg.retrieveValue<unsigned int>("./minNumSamples",10);
			unsigned int maxVariance=cfg.retrieveValue<unsigned int>("./maxVariance",2);
			float hysteresis=cfg.retrieveValue<float>("./hysteresis",0.1f);
			
			//Create the frame filter object:
			frameFilter=new FrameFilter(frameSize,numAveragingSlots,pixelDepthCorrection,cameraIps.depthProjection,basePlane);
			frameFilter->setValidElevationInterval(cameraIps.depthProjection,basePlane,elevationRange.getMin(),elevationRange.getMax());
			frameFilter->setStableParameters(minNumSamples,maxVariance);
			frameFilter->setHysteresis(hysteresis);
			frameFilter->setSpatialFilter(true);
			frameFilter->setOutputFrameFunction(Misc::createFunctionCall(this,&Sandbox::receiveFilteredFrame));

			bool depthData = false;

			if(filteredFrames.lockNewValue())
			{
				// Update the depth image renderer's depth image:
				depthImageRenderer->setDepthImage(filteredFrames.getLockedValue());

				typedef GLfloat DepthPixel;
				Kinect::FrameBuffer fb = filteredFrames.getLockedValue();

				if(!depthData){
					const DepthPixel *dfPtr = fb.getData<DepthPixel>();
					const int x = fb.getSize()[0];
					const int y = fb.getSize()[1];
					GLfloat depthArray[x][y];
					for (unsigned int i = 0; i < x * y; ++i, ++dfPtr) {
						int xVal = i % x;
						int yVal = floor(i / x);
						depthArray[xVal][yVal] = *dfPtr;
						std::cout << "(" << xVal << ", " << yVal << ") = "<< depthArray[xVal][yVal] << std::endl;
					}
					depthData = true;
				}
			}
			*/	

		
		}

		if(buttonSlotIndex==1){
			std::cout << "Volume end" << std::endl;
		}
		}
	else // Button was just released
		{
			std::cout << "button released" << std::endl;
		}
	}
