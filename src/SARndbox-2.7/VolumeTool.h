
#ifndef VOLUMETOOL_INCLUDED
#define VOLUMETOOLL_INCLUDED

#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>

/* Forward declarations: */
class Sandbox;
class VolumeTool;
typedef Vrui::GenericToolFactory<VolumeTool> VolumeToolFactory;

class VolumeTool:public Vrui::Tool,public Vrui::Application::Tool<Sandbox>
	{
	friend class Vrui::GenericToolFactory<VolumeTool>;
	
	/* Elements: */
	private:
	static VolumeToolFactory* factory; // Pointer to the factory object for this class
	
	/* Constructors and destructors: */
	public:
	static VolumeToolFactory* initClass(Vrui::ToolManager& toolManager);
	VolumeTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~VolumeTool(void);
	
	/* Methods from class Vrui::Tool: */
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	};

#endif
