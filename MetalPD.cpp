#include "MetalPD.h"

MetalPD::MetalPD(float pGainIn, float dGainIn)
{
        /* Initialize all PID settings */
		ds = DriverStationLCD::GetInstance();
        pGain = pGain;
        dGain = dGain;
        dState = 0;
        modifiedReference		= 0;
        previousReference		= 0;
        actualPreviousReference	=	0;
        constantReference		= 0;
        constantError			= 0;
        pidTimer	= new Timer();
        refChanged	= false;
}

float MetalPD::Update(float reference, float position)
{
        float pTerm, dTerm;
        float error, output;

        if(previousReference != reference)
        	constantReference = position;

        if(actualPreviousReference != reference)
        {
        	constantError	  = fabs(reference - position)/20 + 2;
        	//ds->PrintfLine(DriverStationLCD::kUser_Line2,"C ERROR : %f",constantError);
        }

        if(modifiedReference != reference && pidTimer->Get() < constantError)
        {
        	pidTimer->Start();

        	/*if(constantReference > reference)
        	{
        		// Going Down
        		modifiedReference = constantReference - (constantReference - reference)*(atan((pidTimer->Get()*4*PI-PI/2)/3) + 1.25)*1.1/3;
        		//modifiedReference = constantReference - (constantReference - reference)*pidTimer->Get()/3;
        		ds->PrintfLine(DriverStationLCD::kUser_Line2,"Going Down");
                // Calculate the error
                error = modifiedReference - position;

                // calculate the proportional term
                pTerm = (pGain * error);

        	}else{
        		// Going Up
        		modifiedReference = constantReference - (constantReference - reference)*(atan((pidTimer->Get()*4*PI-(2*PI*PI))/3) + 1.5)*1.1/3;
        		ds->PrintfLine(DriverStationLCD::kUser_Line2,"Going Up");
                // Calculate the error
                error = modifiedReference - position;

                // calculate the proportional term
                pTerm = (pGain * error);
        	}*/
        	modifiedReference = constantReference - (constantReference - reference)*(atan((pidTimer->Get()*4*PI-(2*PI*PI))/constantError) + (1.48-(7/130*(constantError-2))))*1.1/constantError;

        }else{
        	pidTimer->Stop();
        	pidTimer->Reset();
        	modifiedReference = reference;


        	/* Calculate the error
        	error = modifiedReference - position;

        	// calculate the proportional term
        	pTerm = (pGain * error);
        	*/
        }

       ///*
       // Calculate the error
        error = modifiedReference - position;

       //if(error > 0.0 && error < -1.0*constantError)
       //{
       //error = error-(error*constantError/2);
       //}
        // calculate the proportional term
        pTerm = (pGain * error);
        //*/

        // calculate the d term
        if((actualPreviousReference - position) >= 0){
        	dTerm = (0.100 * (position - dState));
		}else{
			dTerm = (dGain * 10 * (position - dState));
		}

        output = pTerm - dTerm;
        // update the d state to our current position
        ds->PrintfLine(DriverStationLCD::kUser_Line3,"Mod Ref : %f",modifiedReference);
        ds->UpdateLCD();
        //previousReference = reference; // this commented out, good up, bad down
        actualPreviousReference = reference;
        dState = position;
        return output;
}

void MetalPD::Set(float pGainIn, float dGainIn)
{
        /* Initialize all pd settings */
        pGain = pGainIn;
        dGain = dGainIn;
        //dState = 0;
}

void MetalPD::Reset()
{
        dState = 0;
        ///previousReference	= 0;
        pidTimer->Stop();
        pidTimer->Reset();
}
