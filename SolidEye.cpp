#include "SolidEye.h"


SolidEye::SolidEye(MetalSensors *sensors)
{
	io = sensors;
	degreesOff = 0;
}

float SolidEye::GetTarget(void)
{
	if(io->camera->IsFreshImage())
	{
		colorImage = io->camera->GetImage();
		BinaryImage *binaryImage = colorImage->ThresholdHSL(60,255,176,255,65,171);
		particles = binaryImage->GetOrderedParticleAnalysisReports();
		if(particles->size() > 0)
		{
			ParticleAnalysisReport particle = particles->at(0);
			degreesOff = -((54.0 / 640.0) * ((binaryImage->GetWidth() / 2.0) - particle.center_mass_x-50));
		}else{
			degreesOff = 0;
		}
		io->driverStationDisplay->PrintfLine(DriverStationLCD::kUser_Line3,"Deg Off : %f",degreesOff);
		io->driverStationDisplay->UpdateLCD();
		delete binaryImage;
	}

	return degreesOff;
}

