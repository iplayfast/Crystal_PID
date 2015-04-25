#ifndef PID_H
#define PID_H
#define LIBRARY_VERSION 1.0.0

#define GUARD_GAIN 10
class PID
{
unsigned long lastTime;
unsigned long sampleTime;
double lastInput;
double lastError;
double integrated_error;
double kp;
double ki;
double kd;
double Result;
double MaxOutput;
double MinOutput;


public:
    PID(PID &ref);
    PID(double p,double i,double d,unsigned long sampletime=1000);
    void Setup(double lowerOutputLimit=0.0,double upperOutputLimit=255.0);
    bool Compute(double SetPoint,double Input,bool ExternalTimed=false);// returns true if new result calculated
    double GetResult();// const;
    void SetTunings(const double *pid) { kp = pid[0]; ki = pid[1]; kd = pid[2]; }
    void SetTunings(double p,double i,double d) { kp = p; ki = i; kd = d; }
    void GetTunings(double *p,double *i,double *d) { *p = kp; *i = ki; *d=kd; }
    void GetTunings(double pid[3]) { pid[0] = kp; pid[1] = ki; pid[2]=kd; }
    void SetSampleTime(unsigned long sampletime) { sampleTime = sampletime; }
    void ClearIntegratedError() { integrated_error=0; } // use this to remove build up of error when out of range
    double GetError() const { return lastError; }
};
class PIDAutoTune
{
    bool autotune;
    double autotuneCount;
    double totalError;
    double BestError;
    double pidchange[3];   // potential Change
    int pidindex;        // working on p or i or d
    int state;	// state of algorthm
public:
       PIDAutoTune();
       void Init();
       bool Tune(PID &pid);
void GetState(bool *Tuning, int *pautotuneCount, double *ptotalError, double *pBestError, double *pPchange, double *pIchange, double *pDchange, int *ppidindex);

};


#endif
