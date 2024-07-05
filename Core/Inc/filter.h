void FilterInit(float a[3], float alt);
void FilterUpdate(float g[3], float a[3], float alt);
extern float globalAccel[3];
extern float velocity[3];
extern float altitude;
extern float baroAlt;
extern float altOffset;

// delayed values
extern float delayedVel;
