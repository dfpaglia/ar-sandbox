
#ifndef VOLUMETOOL_INCLUDED
#define VOLUMETOOL_INCLUDED

#include <Geometry/Point.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>

#include "RawKinectViewer.h"

/* Forward declarations: */
class RawKinectViewer;

class VolumeTool;
typedef Vrui::GenericToolFactory<VolumeTool> VolumeToolFactory;

class VolumeTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
	{
	friend class Vrui::GenericToolFactory<VolumeTool>;
	
	/* Embedded classes: */
	private:
	typedef Geometry::Point<double,2> Point;
	
	/* Elements: */
	private:
	static VolumeToolFactory* factory; // Pointer to the factory object for this class
	
	bool dragging; // Flag whether the tool is currently dragging a rectangle
	bool button2;
	Point p0; // The rectangle's initial corner
	Point p1; // The rectangle's dragged corner
	int xSize;
	int ySize;
	double xValEnd;
	double yValEnd;

	struct volumetricPoint
	{
		RawKinectViewer::CPoint volumePoint;
	};

	struct volumetricPoint **depthArray;
	struct volumetricPoint **depthArray2;
	
	/* Constructors and destructors: */
	public:
	static VolumeToolFactory* initClass(Vrui::ToolManager& toolManager);
	VolumeTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~VolumeTool(void);
	
	/* Methods from class Vrui::Tool: */
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

#endif
