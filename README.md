pwm(freq,50,1,1,pul_ser);
/*Servo 1*/
int8_t pul_ser[6];
unsigned int cnt;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	cnt++;
	if(cnt == pul_ser[0]){
		PWM_out(freq,0,&htim4,4);
		cnt=0;}
		
		if(cnt == pul_ser[1]){
		PWM_out(freq,0,&htim4,4);
		cnt=0;}
			
}
void controlservo()
{
	// ngat
	
}

void PWM_out(uint32_t freq, int16_t duty, TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t numberOfPulse[6])
{	
	int i = 0;
	for ( i = 0; i < 6; i++ )
	{	
		pul_ser[i] = numberOfPulse[i];
	}	
	uint32_t period;
	
	period = 84000000/(freq*(htim->Instance->PSC+1));
	ARR = period + 1;
	
	period = 84000000/(freq*(htim->Instance->PSC+1))-1;	
	if (period > 0xffff)
		period = 0xffff;
	
	htim->Instance->ARR = period;
	if (duty<-1000)
		duty = -1000;
	else if (duty > 1000)
		duty = 1000;

 if (numberOfPulse > 0)	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // dir = 1
		
	}
	else 	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // dir = 0
		numberOfPulse= -numberOfPulse;
	}
	
	duty = (htim->Instance->ARR+1)*duty/1000;
	
	period  -<-> ARR+1
	duty	<-> CCR
	
	CCR = 
	switch(Channel){
		case 1:	
			htim->Instance->CCR1 = (uint32_t)duty;
		break;
		case 2:
			htim->Instance->CCR2 = (uint32_t)duty;
		break;
		case 3:
			htim->Instance->CCR3 = (uint32_t)duty;
		break;
		case 4:
			htim->Instance->CCR4 = (uint32_t)duty;
		break;
		default:
		break;
	}
}
