task trackPos(){

	float chassisWidth /* = */ ;
	float dR;
	float dL;
	float dM;
	float dS;
	float dTheta;
	float avgTheta;
	float dX;
	float dY;

	float lastEncoderValueL = 0;
	float lastEncoderValueR = 0;
	float lastEncoderValueM = 0;
	float currentEncoderValueL = 0;
	float currentEncoderValueR = 0;
	float currentEncoderValueM = 0;

	while(true){

		currentEncoderValueR = SensorValue[rTracker];
		currentEncoderValueL = SensorValue[lTracker];

		// Can multiply by a scalar to get the values in inches, or whatever unit I want x and y coordinates to be in.
		dR = currentEncoderValueR - lastEncoderValueR;
		dL = currentEncoderValueL - lastEncoderValueL;
		dM = currentEncoderValueM - lastEncoderValueM;

		dR *= ((2.875 * PI) / 360);
		dL *= ((2.875 * PI) / 360);
		dM *= ((2.875 * PI) / 360);

		dTheta = (dR - dL) / chassisWidth;  // Change in angle robot turned since last checked.
		dS *= 2.0 * sin(dTheta / 2.0) / dTheta;  // arc based distance robot traveled since last checked.

		avgTheta = theta + dTheta / 2.0;  // Angle robot is assumed to have been facing when moving dS.

		// Change in x and y position since last checked.
		dX = dS * cos(avgTheta) + dM * sin(avgTheta);
		dY = dS * sin(avgTheta) - dM * cos(avgTheta);

		// Update current robot position.
		xPos += dX;
		yPos += dY;
		theta += dTheta;

		lastEncoderValueL = currentEncoderValueL;
		lastEncoderValueR = currentEncoderValueR;
		lastEncoderValueM = currentEncoderValueM;

		writeDebugStreamLine("X = %f   Y = %f   Heading = %f", xPos, yPos, theta);

		delay(5);
	}

}

void moveTo(int targetX, int targetY){

	float distFromTarget = sqrt( pow((targetX - xPos),2) + pow((targetY - yPos),2) );
	float angleFromTarget = atan2((yPos - targetY),(xPos - targetX)) - theta;
	float drivePower = 0;
	float turnPower = 0;
	float highPass = 50;

	struct PID drivePID;
	initPIDStruct(&drivePID, 0, 0, 0);
	struct PID turnPID;
	initPIDStruct(&turnPID, 0, 0, 0);

	/*
	float deltaTrackerR = 0;
	float deltaTrackerL = 0;
	float frozenTrackerValR = rTracker;
	float frozenTrackerValL = lTracker;
	*/

	/*
	minigoal for deltaTracker: get value that mimics what the encoder is getting but starting at 0 without resetting encoder
															so i dont mess with trackPos()
	*/
	while(true){

		angleFromTarget = atan2((yPos - targetY),(xPos - targetX)) - theta;
		distFromTarget = sqrt( pow((targetX - xPos),2) + pow((targetY - yPos),2) );

		//deltaTrackerR = SensorValue[rTracker] - frozenTrackerValR;
		//deltaTrackerL = SensorValue[lTracker] - frozenTrackerValL;

		/*
		if((deltaTrackerR + deltaTrackerL)/2 == targetDist){
			break;
		}
		*/

		drivePower = calcPID(&drivePID, distFromTarget, 0);
		turnPower = calcPID(&turnPID, angleFromTarget, 0);

		if(turnPower > highPass){
			turnPower = highPass;
		}

		powerDrive(drivePower + turnPower, drivePower - turnPower);

		delay(10);
	}

}
