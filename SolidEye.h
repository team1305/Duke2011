/*----------------------------------------------------------------------------*/
/* Vision class / tracking functions										  */
/*----------------------------------------------------------------------------*/
#ifndef SOLIDEYE_H_
#define SOLIDEYE_H_
#include "WPILib.h"
#include "MetalSensors.h"
#include "nivision.h"
#include "Vision/ColorImage.h"
#include "Vision/BinaryImage.h"
#include "Vision/Threshold.h"
#include <algorithm>
#include <vector>
#include <math.h>

class SolidEye
{
public:
	SolidEye(MetalSensors *sensors);
	float GetTarget(void);
	AxisCamera 		*camera;
	float degreesOff;
private:
	bool cameraInitialized;
	MetalSensors *io;
	ColorImage *colorImage;
	vector<ParticleAnalysisReport> *particles;
};

#endif
