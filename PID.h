float calcPID(struct PID *PID, float input, float target){ //calculates the PID values for any PID struct passed to it by reference, outputs power

	//while((SensorValue[rTracker] + SensorValue[lTracker])/2 < target){

		PID->error = target - input;

		PID->integral += PID->error * PID->dT;

		PID->derivative = (PID->error - PID->prevError)/PID->dT;

		PID->derivative = (PID->lastDerivative * .5) + (PID->derivative * .5);

		PID->power = (PID->kP * PID->error) + (PID->kD * PID->derivative) + (PID->integral * PID->kI);

		PID->prevError = PID->error;

		PID->lastDerivative = PID->derivative;

		return PID->power;

	//}
}

void initPIDStruct(struct PID *PID, float kP, float kI, float kD){ //function used to initialize a PID struct with the PID constants and sets everything else to 0
	PID->kP = kP;
	PID->kI = kI;
	PID->kD = kD;

	PID->error = 0;
	PID->prevError = 0;
	PID->derivative = 0;
	PID->integral = 0;
	PID->lastDerivative = 0;

	PID->power = 0;
	PID->dT = 10;

}
