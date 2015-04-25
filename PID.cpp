/**********************************************************************************************
 * Arduino PID Library - Version 1.0.0
 * by Chris Bruner
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include <math.h>
#include "PID.h"

//#define DEBUG_PIDTUNE
//#define DEBUG_PID
// number of samples before applying tunings
#define TUNEAVG 50
PID::PID(double p,double i,double d,unsigned long sampletime)
{
    kp = p;
    ki = i;
    kd = d;
    sampleTime = sampletime;
}

void PID::Setup(double lowerOutputLimit,double upperOutputLimit)
{
    lastTime = millis();
    integrated_error = 0.0;
    lastInput = 0;
    Result = 0;
	MaxOutput = upperOutputLimit;
	MinOutput = lowerOutputLimit;
}
PID::PID(PID &ref)
{
    lastTime= ref.lastTime;
    sampleTime= ref.sampleTime;
    lastInput= ref.lastInput;
    kp= ref.kp;
    ki= ref.ki;
    kd= ref.kd;
    Result= ref.Result;
    integrated_error = ref.integrated_error;
    MaxOutput= ref.MaxOutput;
    MinOutput= ref.MinOutput;
}
#include "SerialPrint.h"
bool PID::Compute(double SetPoint,double Input,bool ExternalTimed)
{

    unsigned long now,timeChange;
    now = millis();
    timeChange = (now - lastTime);
    if (timeChange<1) return false; //  even if externally timed, this is too fast
    if(ExternalTimed || timeChange>=sampleTime)
    {
        lastTime = now;
        /*Compute all the working error variables*/
    double error =SetPoint - Input;
//SerialPrint("Timechange",timeChange);
            integrated_error += error * timeChange;
//SerialPrint("ie ",integrated_error);
            double derivative = (error - lastError) / timeChange;
            Result = kp * error + ki * integrated_error + kd*derivative;
            lastError = error;
            if (Result<MinOutput) Result = MinOutput;
            if (Result>MaxOutput) Result = MaxOutput;
#ifdef DEBUG_PID
        Serial.print("pTerm ");Serial.print(pTerm);
        Serial.print(" iTerm ");Serial.print(iTerm);
        Serial.print(" dTerm ");Serial.print(dTerm);
        Serial.print(" result "); Serial.println(Result);
#endif
        return true;
    }
    else return false;
}

double PID::GetResult() //const
{
    if (isnan(Result)){
	Result =0;
		
        return 55;
	}
    return Result;
}

PIDAutoTune::PIDAutoTune()
{

}


void PIDAutoTune::Init()
{
    autotune = true;
    autotuneCount = 0;
    totalError = 0.0;
    pidchange[0] = 0.1; pidchange[1] = 0.001; pidchange[2] = 0.0001;
    pidindex = 0;
    state = 0;
    BestError = 1000000000;
}

bool PIDAutoTune::Tune(PID &pid)
{
    if (!autotune) return false;  // not autotuning
    double error = pid.GetError();
        //get error, we don't care about direction, only amount
    totalError +=  (error<0)? -error : error;
    autotuneCount++;
    if (autotuneCount<TUNEAVG) return false;
    double averageError =  totalError / autotuneCount; // get average error over TUNEAVG tests
averageError /=4; // get rid of some noise
    autotuneCount = 0;
    totalError =0;

    double kpid[3];
    pid.GetTunings(kpid); // get old tunings

#ifdef DEBUG_PIDTUNE
Serial.print("BestError "); Serial.print(BestError);
Serial.print("averageError "); Serial.print(averageError);
#endif
switch(state) {
	case 0:
		kpid[pidindex] += pidchange[pidindex];	// adjusting tunings a bit
		pid.SetTunings(kpid);
		state++;
		break;
	case 1:
		if (averageError<BestError)   {	// we have a better tuning
			BestError = averageError;
			pidchange[pidindex] *= 1.1;		// adjust to make a larger change
			kpid[pidindex] += pidchange[pidindex];	// do state 0's stuff here again
			pid.SetTunings(kpid);		// state stays the same until we get worse
		}
		else {	// last change wasn't better
			kpid[pidindex] -= 2.0 * pidchange[pidindex];	// take off last change twice
			state = 2;
		}
		break;
	case 2:
		if (averageError>BestError) {	// last change wasn't better
			kpid[pidindex] += pidchange[pidindex];	// add last change back
			pidchange[pidindex] *= 0.9;	// reduce the amount of change
			state = 0;
			pidindex++;
			if (pidindex>2) pidindex = 0;
			break;
		}
}
#ifdef DEBUG_PIDTUNE
Serial.print(" change pidindexs p"); Serial.print(pidchange[0]);
Serial.print(" i"); Serial.print(pidchange[1]);
Serial.print(" d"); Serial.print(pidchange[2]);
Serial.print("PID values p"); Serial.print(kpid[0]);
Serial.print(" i"); Serial.print(kpid[1]);
Serial.print(" d"); Serial.println(kpid[2]);
#endif
    pid.SetTunings(kpid);
    if (pidchange[0] +pidchange[1] + pidchange[2] < 0.0001  )
        autotune =false;
    return true;
}

void PIDAutoTune::GetState(bool *Tuning, int *pautotuneCount, double *ptotalError, double *pBestError, double *pPchange, double *pIchange, double *pDchange, int *ppidindex)
{
    *Tuning = autotune;
    *pautotuneCount = autotuneCount;
    *ptotalError = totalError;
    *pBestError = BestError;
    *pPchange = pidchange[0];
    *pIchange = pidchange[1];
    *pDchange = pidchange[2];
    *ppidindex = pidindex;
}

