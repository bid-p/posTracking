struct PID{ //struct definition for PIDs

	float kP;
	float kI;
	float kD;

	float error;
	float prevError;
	float derivative;
	float integral;
	float lastDerivative;

	int power;
	float dT;

};
