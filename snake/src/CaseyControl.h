#include<vector>
class CaseyControl{
private:
	double positionVector[3];
	double velocityVector[3];
    double effortVector[3];

	
public:
	CaseyControl();
	~CaseyControl();

    double getPosition(int i);
    double getVelocity(int i);
	double getEffort(int i);
	
	void setPosition(int i, double p);
	void setVelocity(int i, double v);
	void setEffort(int i, double e);
    
    void PitchPID(double p);
    void YawPID(double y);
    void PitchTorqueInput(double p);
    void YawTorqueInput(double y);
    
    void FillZeroes();
};
