void Side_Calibrate(); //for sending side walls
void Front_Calibrate(); //for sensing front walls
void Deadend_Calibrate(); //for moving to the end of the dead end
void Correction_Calibrate();
void Adjuster();


void Forward_Search_New();
void Right_Search_New();
void Left_Search_New();
void Dead_End_New();
void Search_New();


int Emergency_Stop();

void Search_New() {

	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, ON); //Turn on LEDs to indicate it is searching
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, OFF);

	HAL_Delay(1000); //delay before start to get finger out of the way

	if (reverse_flag == FALSE) {
		Reset_Maze(); //reset walls
		Read_Walls();
	}

	Clear_Buffers();
	Reset_Counters();

	Reset_Flags(); //reset flags
	Floodfill(reverse_flag, search_flag, FALSE);

	Update_Position_New(); //move to next square (first move is always forward)

	Set_Left(FWD_L, FORWARD); //start going straight. get moving
	Set_Right(FWD_R, FORWARD);

	while(1) {

		switch(cur_move) { //perform whatever move is queued
		case FWD:
			Forward_Search_New();
			break;

		case RIGHT: //break up turn into turn and accelerate
			Right_Search_New();
			break;

		case LEFT:
			Left_Search_New();
			break;

		case DEAD:
			Dead_End_New();
			break;
		} //walls are read during the state machine

		prev_l_count = l_count; //get current counter values
		prev_r_count = r_count;

		if (maze[x_coord][y_coord] == 0) { //reached target square
			if (search_flag == FALSE) { //reached center
				sprintf(tx_buffer, "|||FINAL||| X VALUE: %d  Y VALUE: %d \r\n", x_coord, y_coord);
				Transmit(tx_buffer);
				Dead_End_Correct();
				Switch_Direction();
				break; //break out of while loop
			}

			else { //looking along for the optimal path still
				Calc_Optimal(); //recalculate optimal path
				Get_Coordinate(); //get any coordinates that haven't been seen yet
				if (coordinate_count == 0) { //if none, go back to start
					search_flag = FALSE;
				}
				Floodfill(reverse_flag, search_flag, FALSE); // go back to center
				next_move = Get_Next_Move(); //queue up next move
				Update_Position_New(); //update the position so walls can be read
				cur_move = next_move;
			}
		}
		else {
			next_move = Get_Next_Move(); //queue up next move
			Update_Position_New(); //update the position so walls can be read
			cur_move = next_move;
		}
	}

}

void Forward_Search_New() {
	Set_Left(FWD_L, FORWARD);
	Set_Right(FWD_R, FORWARD);
	transition_flag = FALSE;

	do {

		Update_Sensors(FWD_SEARCH);
		if (transition_flag == FALSE && (r_transition_flag == TRUE || l_transition_flag == TRUE)) { //detect transition and start counting from there

			prev_l_count = l_count;
			prev_r_count = r_count;
			transition_flag = TRUE;
			lenc_diff = 0;
			renc_diff = 0;

			if (l_transition_flag == TRUE) {
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ON);
			}
			else if (r_transition_flag == TRUE) {
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ON);
			}
		}

		Search_Correction();

	} while ((transition_flag == FALSE && lenc_diff < F_ENC1 && renc_diff < F_ENC1) || (transition_flag == TRUE && lenc_diff < FWD_TRANS/2 && renc_diff < FWD_TRANS/2));

	Read_Walls();
	Save_State();

	do {
		Update_Sensors(BARE);
		Search_Correction();

	} while ((transition_flag == FALSE && lenc_diff < F_ENC2 && renc_diff < F_ENC2) || (transition_flag == TRUE && lenc_diff < FWD_TRANS && renc_diff < FWD_TRANS));

	Set_Left(0, FORWARD);
	Set_Right(0, FORWARD);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);


}

void Left_Search_New() {
	Set_Left(LEFT_L, FORWARD);
	Set_Right(LEFT_R, FORWARD);

	do {
		r_count = __HAL_TIM_GET_COUNTER(&htim4);
		renc_diff = r_count - prev_r_count;
	} while (renc_diff < LT_RENC_1);

	prev_l_count = l_count;
	prev_r_count = r_count;

	Set_Left(FWD_L, FORWARD);
	Set_Right(FWD_R, FORWARD);

	do {
	Update_Sensors(TURN_SEARCH);
	Search_Correction();
	} while (lenc_diff < LT_ENC_2*2/3 && renc_diff < LT_ENC_2*2/3);

	Read_Walls();
	Save_State();

	do {
	Update_Sensors(TURN_SEARCH);
	Search_Correction();
	} while (lenc_diff < LT_ENC_2 && renc_diff < LT_ENC_2);

	Set_Left(0, FORWARD);
	Set_Right(0, FORWARD);
}

void Right_Search_New() {
	Set_Left(RIGHT_L, FORWARD);
	Set_Right(RIGHT_R, FORWARD);

	do {
		l_count = __HAL_TIM_GET_COUNTER(&htim1);
		lenc_diff = l_count - prev_l_count;
	} while (lenc_diff < RT_LENC_1);

	prev_l_count = l_count;
	prev_r_count = r_count;

	Set_Left(FWD_L, FORWARD);
	Set_Right(FWD_R, FORWARD);

	do {
	Update_Sensors(TURN_SEARCH);
	Search_Correction();
	} while (lenc_diff < RT_ENC_2*2/3 && renc_diff < RT_ENC_2*2/3);

	Read_Walls();

	Save_State();

	do {
	Update_Sensors(TURN_SEARCH);
	Search_Correction();
	} while (lenc_diff < RT_ENC_2 && renc_diff < RT_ENC_2);

	Set_Left(0, FORWARD); //brake
	Set_Right(0, FORWARD);

}

void Read_Gyro(void) {

	for (int i = 0; i < 2; i++) {
		HAL_GPIO_WritePin(GPIOA, GYRO_CS_Pin, GPIO_PIN_RESET); //assert CS pin
		if(HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 4) != HAL_OK)
		{
			/* Transfer error in transmission process */
			Error_Handler();
		}
		HAL_GPIO_WritePin(GPIOA, GYRO_CS_Pin, GPIO_PIN_SET);
		}
	gyro_reading = *aRxBuffer;
	sprintf(tx_buffer, "Gyro Reading: %X \r\n", gyro_reading);
	Transmit(tx_buffer);
}

void Deadend_Calibrate() {

	Clear_Buffers();
	buff_count = 0;
	int l_test = 0;
	int r_test = 0;

	for (int i = 0; i < IR_BUFFER; i++) {
		Get_IR(TRUE, FALSE, FALSE, TRUE);
	}

	for (int i = 0; i < IR_BUFFER; i++) {

	l_test = l_test + l_buffer[i];
	r_test = r_test + r_buffer[i];

	}

	l_test = l_test/IR_BUFFER;
	r_test = r_test/IR_BUFFER;

	sprintf(tx_buffer, "DeadEnd: \r\nLEFT: %d    RIGHT: %d      OFFSET(LEFT - RIGHT): %d \r\n", l_test, r_test, l_test - r_test);
	Transmit(tx_buffer);
}

void Correction_Calibrate() {

#define VAL 20

Read_Settings();
Reset_Counters();

int val;

//modify offsets


HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ON);

stop_flag = TRUE;
do {
r_count = __HAL_TIM_GET_COUNTER(&htim4);
l_count = __HAL_TIM_GET_COUNTER(&htim1);
sprintf(tx_buffer, "LEFT CORRECTION VALUE: %d \r\n", offsets[0] + l_count/VAL - r_count/VAL);
if (l_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
}
if (r_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

Transmit(tx_buffer);
HAL_Delay(1000);
} while (stop_flag == TRUE);

val = offsets[0] + l_count/VAL - r_count/VAL;
if (abs(val) < 2000) {
	offsets[0] = val;
}

HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, OFF);
HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ON);
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);

stop_flag = TRUE;
Reset_Counters();
do {
r_count = __HAL_TIM_GET_COUNTER(&htim4);
l_count = __HAL_TIM_GET_COUNTER(&htim1);
sprintf(tx_buffer, "RIGHT CORRECTION VALUE: %d \r\n", offsets[1] + l_count/VAL - r_count/VAL);
if (l_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
}
if (r_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}
Transmit(tx_buffer);
HAL_Delay(1000);
} while (stop_flag == TRUE);

val = offsets[1] + l_count/VAL - r_count/VAL;
if (abs(val) < 2000) {
	offsets[1] = val;
}

HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, ON);
HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, OFF);
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, OFF);

Reset_Counters();
stop_flag = TRUE;
do {
r_count = __HAL_TIM_GET_COUNTER(&htim4);
l_count = __HAL_TIM_GET_COUNTER(&htim1);
if (l_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
}
if (r_count % 100 == 0) {
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}
sprintf(tx_buffer, "WALL OFFSET VALUE: %d \r\n", offsets[2] + l_count/VAL - r_count/VAL);
Transmit(tx_buffer);
HAL_Delay(1000);
} while (stop_flag == TRUE);

val = offsets[2] + l_count/VAL - r_count/VAL;
if (abs(val) < 2000) {
	offsets[2] = val;
}
Program_Settings();

/*
HAL_GPIO_WritePin(GPIOD, LED5_Pin, OFF); //Turn on LEDs to indicate it is searching
HAL_GPIO_WritePin(GPIOD, LED2_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED4_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED3_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED1_Pin, ON);

stop_flag = TRUE;
do {
	Get_IR(FALSE, FALSE, FALSE, FALSE);
} while (stop_flag == TRUE);


HAL_GPIO_WritePin(GPIOD, LED5_Pin, ON); //Turn on LEDs to indicate it is searching
HAL_GPIO_WritePin(GPIOD, LED2_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED4_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED3_Pin, OFF);
HAL_GPIO_WritePin(GPIOD, LED1_Pin, OFF);

HAL_Delay(1000); //delay before start to get finger out of the way

for (int i = 0; i < 2; i++) {

	Set_Left(FWD_L, FORWARD);
	Set_Right(FWD_R, FORWARD);
	Reset_Counters();

	do {
	Update_Sensors(BARE);
	Search_Correction();
	} while (lenc_diff < F_ENC2*4 && renc_diff < F_ENC2*4);

	Dead_End_Correct();
}
Stop();
*/


}

void Adjuster() {

int left = 0;
int right = 0;
int l_dic = FORWARD;
int r_dic = FORWARD;

left = 2500 - dif_l;
if (left < 0) {

	l_dic = BACKWARD;
	left = abs(left);
}

right = 2500 - dif_r;
if (right < 0) {

	r_dic = BACKWARD;
	right = abs(right);
}

Set_Left(left, l_dic);
Set_Right(right, r_dic);

}

void Turn_Profiler(int *speeds, int direction, int num_squares) {

	//inside wheel speed 1 (same as 3)
	//outside wheel speed 1 (same as 3)
	//inside wheel speed 2
	//outside wheel speed 2
	//outside wheel distance
	//inside wheel distance

	//uint32_t left_speed = 180*1000000/700*(lenc_diff - temp_l)/(time_count - prev_time_count); // in mm/s
	//uint32_t right_speed = 180*1000000/700*(renc_diff - temp_r)/(time_count - prev_time_count); // in mm/s

	/*if (inside_wheel distance < inside threshold) {
	 correction}
	 else if (inside_wheel distance < inside threshold 2) { speed correction
	 }
	 else { speed correction
	 }
	 */
}
/*
void Speed_Profiler(uint32_t *speeds, int num_squares, int distance) {


	 * SPEED/ACCeleration CALCULATION

	static const uint32_t brake_distance[15] = //in counts
	{
		140, 140, 140, 140, 140, //FWD1 - FWD5
		140, 140, 140, 140, 140, //FWD6 - FWD10
		140, 140, 140, 140, 140 //FWD11 - FWD15
	};

	Update_Sensors(FWD_SEARCH);

	static int center_speed = 0; //average of left and right values in mm/s
	static int left_speed = 0; //mm/s
	static int right_speed = 0; //mm/s
	static int left_acceleration = 0; //speed - prev_speed / time - prev_time
	static int right_acceleration = 0;
	static uint32_t prev_left_speed = 0; //variable to store previous speed
	static uint32_t prev_right_speed = 0;
	if (time_count - prev_time_count > 1000) { //sample every ms

		if (prev_time_count > time_count) { //overflow
			int diff = prev_time_count - 0xffffffff;
			prev_time_count = (unsigned int) diff;
		}

		left_speed = 180*1000000/700*(l_count - temp_l)/(time_count - prev_time_count); // in mm/s
		right_speed = 180*1000000/700*(r_count - temp_r)/(time_count - prev_time_count); // in mm/s
		center_speed = (left_speed+right_speed)/2;
		left_acceleration = 1000000*(left_speed - prev_left_speed)/(time_count - prev_time_count);
		right_acceleration = 1000000*(right_speed - prev_right_speed)/(time_count - prev_time_count);

		if (center_speed > top_speed) {
			top_speed = center_speed;
		}
		temp_l = l_count;
		temp_r = r_count;
		prev_time_count = time_count;
		prev_left_speed = left_speed;
		prev_right_speed = right_speed;

	}



	static uint32_t distance_left; //distance left to cover in counts
	distance_left = distance - (lenc_diff + renc_diff)/2;

	m_correction = Motor_Correction(FALSE); //motor correction

	if (distance_left <= F_ENC2) {
		Detect_Transition(); //only update values, don't actually turn on anything
	}

	speeds[1] = FWD_L + m_correction;
	speeds[2] = FWD_R - m_correction;

 //distance left

	if (distance_left < distance/4) { //decelerate to end speed
		Get_IR(FALSE, TRUE, TRUE, FALSE);
		Detect_Transition();
		m_correction = Motor_Correction(FALSE); //motor correction

		speeds[1] = FWD_L + m_correction;
		speeds[2] = FWD_R - m_correction;
		//brake until a certain speed reached
	}

	else if (distance_left > distance*3/4) { //accelerate to top speed with motor correction

		Get_IR(FALSE, TRUE, TRUE, FALSE);
		m_correction = Motor_Correction(FALSE); //motor correction with infrared
		speeds[1] = speeds[0]/2 + m_correction; //return speeds
		speeds[2] = speeds[0]/2 - m_correction;
	}

	else if (distance_left > distance/2 && distance_left <= distance*3/4) { //accelerate to top speed with motor correction

		Get_IR(FALSE, TRUE, TRUE, FALSE);
		m_correction = Motor_Correction(FALSE); //motor correction with infrared
		speeds[1] = speeds[0]*2/3 + m_correction; //return speeds
		speeds[2] = speeds[0]*2/3 - m_correction;
	}

	else if (distance_left > distance/4 && distance_left < distance/2) { //accelerate to top speed with motor correction

		Get_IR(FALSE, TRUE, TRUE, FALSE);
		m_correction = Motor_Correction(FALSE); //motor correction with infrared
		speeds[1] = speeds[0] + m_correction; //return speeds
		speeds[2] = speeds[0] - m_correction;
	}



}

*/


void Dead_End_New() {

	Dead_End_Correct(); //should be aligned in middle of square in the back

	Floodfill(reverse_flag, search_flag, FALSE); //floodfill maze
	Print_Maze();
	Reset_Counters();
	Clear_Buffers();

	Set_Left(FWD_L, FORWARD);
	Set_Right(FWD_R, FORWARD); //move forward to next square

	do {
		Update_Sensors(TURN_SEARCH);
		Search_Correction();
	} while (lenc_diff >= 650 || renc_diff >= 650);

}

int Emergency_Stop() {

/*if (dif_l > STOP_CONDITION || dif_r > STOP_CONDITION || cur_move == PAUSE) //Emergency STOP conditions
{
	  if (cur_move == FWD || cur_move == PAUSE) //|| l_turnflag == TRUE || r_turnflag == TRUE)
	  {
		  return TRUE;
	  }
}
*/
if (cur_move == PAUSE) {
	return TRUE;
}
return FALSE;
}


void Update_Position_New() {

	visited_squares[x_coord][y_coord] = TRUE;

	if (next_move != DEAD) {
		prevx = x_coord; //save previous values. Used for dead end
		prevy = y_coord;
	}

	switch (cur_dir) {
	case NORTH:
		switch(next_move) {
		case FWD:
			x_coord = x_coord + NORTH_X;
			y_coord = y_coord + NORTH_Y;
			break;
		case LEFT:
			x_coord = x_coord + WEST_X;
			y_coord = y_coord + WEST_Y;
			cur_dir = WEST;
			break;
		case RIGHT:
			x_coord = x_coord + EAST_X;
			y_coord = y_coord + EAST_Y;
			cur_dir = EAST;
			break;
		case DEAD:
			cur_dir = SOUTH;
			break;
		}
	break;

	case SOUTH:
		switch(next_move) {
		case FWD:
			x_coord = x_coord + SOUTH_X;
			y_coord = y_coord + SOUTH_Y;
			break;
		case LEFT:
			x_coord = x_coord + EAST_X;
			y_coord = y_coord + EAST_Y;
			cur_dir = EAST;
			break;
		case RIGHT:
			x_coord = x_coord + WEST_X;
			y_coord = y_coord + WEST_Y;
			cur_dir = WEST;
			break;
		case DEAD:
			cur_dir = NORTH;
			break;
		}
	break; //bitch retard enema asshole kockface
		case WEST:
		switch(next_move) {
		case FWD:
			x_coord = x_coord + WEST_X;
			y_coord = y_coord + WEST_Y;
			break;
		case LEFT:
			x_coord = x_coord + SOUTH_X;
			y_coord = y_coord + SOUTH_Y;
			cur_dir = SOUTH;
			break;
		case RIGHT:
			x_coord = x_coord + NORTH_X;
			y_coord = y_coord + NORTH_Y;
			cur_dir = NORTH;
			break;
		case DEAD:
			cur_dir = EAST;
			break;
		}
		break;

		case EAST:
			switch(next_move) {
			case FWD:
				x_coord = x_coord + EAST_X;
				y_coord = y_coord + EAST_Y;
				cur_dir = EAST;
				break;
			case LEFT:
				x_coord = x_coord + NORTH_X;
				y_coord = y_coord + NORTH_Y;
				cur_dir = NORTH;
				break;
			case RIGHT:
				x_coord = x_coord + SOUTH_X;
				y_coord = y_coord + SOUTH_Y;
				cur_dir = SOUTH;
				break;
			case DEAD:
				cur_dir = WEST;
				break;
			}
			break;
		}

		if (next_move == DEAD) {
			sprintf(tx_buffer, "|||DEAD END||| X VALUE: %d  Y VALUE: %d \r\n", x_coord, y_coord);
			Transmit(tx_buffer);
			x_coord = prevx;
			y_coord = prevy;

		}

		if (x_coord < 0 || y_coord < 0 || x_coord >= X_MAZE_SIZE || y_coord >= Y_MAZE_SIZE) {
			stop_flag = TRUE;
			sprintf(tx_buffer, "Position out of Boundary! X VALUE: %d  Y VALUE %d  \r\n", x_coord, y_coord);
			Transmit(tx_buffer);
		}

		sprintf(tx_buffer, "|||DECISION  %d||| X VALUE: %d  Y VALUE: %d  DIRECTION: %d  NEXT: %d \r\n", dbg_count + 1, x_coord, y_coord, cur_dir, next_move);
		Transmit(tx_buffer);

}

int Get_Next_Dumb() {

int next = 0;

	switch (cur_move) {

		  case FWD:
			  if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && rf_side <= RIGHT_THRESHOLD) //if front and right side is not blocked
			  {next = RIGHT;}
			  else if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && lf_side <= LEFT_THRESHOLD) //if front and right side is blocked, but left is not
			  {next = LEFT;}
			  else if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && lf_side > LEFT_THRESHOLD && rf_side > RIGHT_THRESHOLD)
			  {next = DEAD;}
			  //the default next_move is the cur_move, so if the front isn't blocked, keep going straight
			  else
			  {next = FWD;}
			  break;

		  case RIGHT:
			  if (front_l <= FRONT_THRESHOLD || front_r <= FRONT_THRESHOLD) //r_turnflag means the second part of the turn
			  {next = FWD;}
			  else if (rf_side >= RIGHT_THRESHOLD && lf_side < LEFT_THRESHOLD)
			  {next = LEFT;}
			  else if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && lf_side > LEFT_THRESHOLD && rf_side > RIGHT_THRESHOLD)
			  {next = DEAD;}
			  else
			  {next = RIGHT;}
			  break;

		  case LEFT:
			  if (front_l <= FRONT_THRESHOLD || front_r <= FRONT_THRESHOLD)
			  {next = FWD;}
			  else if (lf_side >= LEFT_THRESHOLD && rf_side < RIGHT_THRESHOLD)
			  {next = RIGHT;}
			  else if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && lf_side >= LEFT_THRESHOLD && rf_side >= RIGHT_THRESHOLD)
			  {next = DEAD;}
			  else
			  {next = LEFT;}
			  break;
		  case DEAD:
			  if (front_l < FRONT_THRESHOLD || front_r < FRONT_THRESHOLD)
			  {next = FWD;}
			  if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && rf_side <= RIGHT_THRESHOLD) //if front and right side is not blocked
			  {next = RIGHT;}
			  else if ((front_l >= FRONT_THRESHOLD || front_r >= FRONT_THRESHOLD) && lf_side <= LEFT_THRESHOLD) //if front and right side is blocked, but left is not
			  {next = LEFT;}
			  break;
		  }

	if (debug_flag == FALSE) //read walls if not being debugged
	{
	switch(cur_dir) {

	case NORTH: //facing up
		if (y_coord > 0 && (front_l > FRONT_THRESHOLD || front_r > FRONT_THRESHOLD)) { //front sensor
			horiz_walls[x_coord][y_coord - 1] = 1; //up wall
		}
		if (x_coord < X_MAZE_SIZE - 1 && rf_side > RIGHT_THRESHOLD) { //right sensor
			vert_walls[x_coord][y_coord] = 1; //right wall
		}
		if (x_coord > 0 && lf_side > LEFT_THRESHOLD) {  //left sensor
			vert_walls[x_coord - 1][y_coord] = 1; //left wall
		}
		break;

	case SOUTH: //facing down
		if (y_coord < Y_MAZE_SIZE - 1 && (front_l > FRONT_THRESHOLD || front_r > FRONT_THRESHOLD)) {  //front sensor
			horiz_walls[x_coord][y_coord] = 1; //down_wall
			}
		if (x_coord > 0 && rf_side > RIGHT_THRESHOLD) {  //right sensor
			vert_walls[x_coord - 1][y_coord] = 1; //left wall
		}
		if (x_coord < Y_MAZE_SIZE - 1 && lf_side > LEFT_THRESHOLD) { //left sensor
			vert_walls[x_coord][y_coord] = 1; //right wall
		}
		break;

	case WEST: //facing left
		if (x_coord > 0 && (front_l > FRONT_THRESHOLD || front_r > FRONT_THRESHOLD)) { //front sensor
			vert_walls[x_coord - 1][y_coord] = 1; //left wall
		}
		if (y_coord > 0 && rf_side > RIGHT_THRESHOLD) {  //right sensor
			horiz_walls[x_coord][y_coord - 1] = 1; //up wall
		}
		if (y_coord < Y_MAZE_SIZE - 1 && lf_side > LEFT_THRESHOLD) {//left sensor
			horiz_walls[x_coord][y_coord] = 1; //down wall
		}
		break;

	case EAST: //facing right
		if (x_coord < X_MAZE_SIZE - 1 && (front_l > FRONT_THRESHOLD || front_r > FRONT_THRESHOLD)) { //front sensor
			vert_walls[x_coord][y_coord] = 1; //right wall
			}
		if (y_coord < Y_MAZE_SIZE - 1 && rf_side > RIGHT_THRESHOLD) { //right sensor
			horiz_walls[x_coord][y_coord] = 1; //down wall
		}
		if (y_coord > 0 && lf_side > LEFT_THRESHOLD) {  //left sensor
			horiz_walls[x_coord][y_coord - 1] = 1; //up wall
		}
		break;

	}
}

return next;
}
