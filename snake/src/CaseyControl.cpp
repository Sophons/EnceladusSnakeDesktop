#include "CaseyControl.h"

CaseyControl::CaseyControl(){
	
}
CaseyControl::~CaseyControl(){

}

double CaseyControl::getPosition(int i){
	return positionVector[i];
}
double CaseyControl::getVelocity(int i){
	return velocityVector[i];
}
double CaseyControl::getEffort(int i){
	return effortVector[i];
}

void CaseyControl::setPosition(int i, double p){
	positionVector[i] = p;
}
void CaseyControl::setVelocity(int i, double v){
	velocityVector[i] = v;
}
void CaseyControl::setEffort(int i, double e){
	effortVector[i] = e;
}

void CaseyControl::PitchPID(double p){}
void CaseyControl::YawPID(double y){}
void CaseyControl::PitchTorqueInput(double p){}
void CaseyControl::YawTorqueInput(double y){}

void CaseyControl::FillZeroes(){
	positionVector[0] = 0.0;
	positionVector[1] = 0.0;
	positionVector[2] = 0.0;

	velocityVector[0] = 0.0;
	velocityVector[1] = 0.0;
	velocityVector[2] = 0.0;

	effortVector[0] = 0.0;
	effortVector[1] = 0.0;
	effortVector[2] = 0.0;
	}
