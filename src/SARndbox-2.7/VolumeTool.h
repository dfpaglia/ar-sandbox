
#ifndef VOLUMETOOL_INCLUDED
#define VOLUMETOOLL_INCLUDED

#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>

/* Forward declarations: */
class Sandbox;
class VolumeTool;
class FrameFilter;
typedef Vrui::GenericToolFactory<VolumeTool> VolumeToolFactory;
typedef Kinect::FrameSource::DepthCorrection::PixelCorrection PixelDepthCorrection; // Type for per-pixel depth correction factors

class VolumeTool:public Vrui::Tool,public Vrui::Application::Tool<Sandbox>
	{
	friend class Vrui::GenericToolFactory<VolumeTool>;

	unsigned int frameSize[2]; // Width and height of the camera's depth frames
	PixelDepthCorrection* pixelDepthCorrection; // Buffer of per-pixel depth correction coefficients
	Kinect::FrameSource::IntrinsicParameters cameraIps; // Intrinsic parameters of the Kinect camera
	FrameFilter* frameFilter; // Processing object to filter raw depth frames from the Kinect camera
	Threads::TripleBuffer<Kinect::FrameBuffer> filteredFrames; // Triple buffer for incoming filtered depth frames
	DepthImageRenderer* depthImageRenderer; // Object managing the current filtered depth image
	
	/* Elements: */
	private:
	void receiveFilteredFrame(const Kinect::FrameBuffer& frameBuffer); // Callback receiving filtered depth frames from the filter object
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
