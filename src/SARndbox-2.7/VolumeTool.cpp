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
