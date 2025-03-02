#include <stdint.h>
#include "VescUart.h"
#include <cstring> // Per memset

VescUart::VescUart(uint32_t timeout_ms) : _TIMEOUT(timeout_ms) {
	nunchuck.valueX         = 127;
	nunchuck.valueY         = 127;
	nunchuck.lowerButton  	= false;
	nunchuck.upperButton  	= false;
}

void VescUart::setSerialPort(Stream* port)
{
	serialPort = port;
}

void VescUart::setDebugPort(Stream* port)
{
	debugPort = port;
}

int VescUart::receiveUartMessage(uint8_t * payloadReceived) {

	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	// Makes no sense to run this function if no serialPort is defined.
	if (serialPort == NULL)
		return -1;

	uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	uint16_t lenPayload = 0;
	
	uint32_t timeout = millis() + _TIMEOUT; // Defining the timestamp for timeout (100ms before timeout)

	while ( millis() < timeout && messageRead == false) {

		while (serialPort->available()) {

			messageReceived[counter++] = serialPort->read();

			if (counter == 2) {

				switch (messageReceived[0])
				{
					case 2:
						endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
						lenPayload = messageReceived[1];
					break;

					case 3:
						// ToDo: Add Message Handling > 255 (starting with 3)
						if( debugPort != NULL ){
							debugPort->println("Message is larger than 256 bytes - not supported");
						}
					break;

					default:
						if( debugPort != NULL ){
							debugPort->println("Unvalid start bit");
						}
					break;
				}
			}

			if (counter >= sizeof(messageReceived)) {
				break;
			}

			if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
				messageReceived[endMessage] = 0;
				if (debugPort != NULL) {
					debugPort->println("End of message reached!");
				}
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}
	if(messageRead == false && debugPort != NULL ) {
		debugPort->println("Timeout");
	}
	
	bool unpacked = false;

	if (messageRead) {
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
	}

	if (unpacked) {
		// Message was read
		return lenPayload; 
	}
	else {
		// No Message Read
		return 0;
	}
}


bool VescUart::unpackPayload(uint8_t * message, int lenMes, uint8_t * payload) {

	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	if(debugPort!=NULL){
		debugPort->print("SRC received: "); debugPort->println(crcMessage);
	}

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if( debugPort != NULL ){
		debugPort->print("SRC calc: "); debugPort->println(crcPayload);
	}
	
	if (crcPayload == crcMessage) {
		if( debugPort != NULL ) {
			debugPort->print("Received: "); 
			serialPrint(message, lenMes); debugPort->println();

			debugPort->print("Payload :      ");
			serialPrint(payload, message[1] - 1); debugPort->println();
		}

		return true;
	}else{
		return false;
	}
}


int VescUart::packSendPayload(uint8_t * payload, int lenPay) {

	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];
	
	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(messageSend + count, payload, lenPay);
	count += lenPay;

	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	// messageSend[count] = NULL;
	
	if(debugPort!=NULL){
		debugPort->print("Package to send: "); serialPrint(messageSend, count);
	}

	// Sending package
	if( serialPort != NULL )
		serialPort->write(messageSend, count);

	// Returns number of send bytes
	return count;
}


bool VescUart::processReadPacket(uint8_t * message) {

	COMM_PACKET_ID packetId;
	int32_t index = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_FW_VERSION: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			fw_version.major = message[index++];
			fw_version.minor = message[index++];
			return true;
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			data.tempMosfet 		= buffer_get_float16(message, 10.0, &index); 	// 2 bytes - mc_interface_temp_fet_filtered()
			data.tempMotor 			= buffer_get_float16(message, 10.0, &index); 	// 2 bytes - mc_interface_temp_motor_filtered()
			data.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_motor_current()
			data.avgInputCurrent 	= buffer_get_float32(message, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_input_current()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_id()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_iq()
			data.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &index); 	// 2 bytes - mc_interface_get_duty_cycle_now()
			data.rpm 				= buffer_get_float32(message, 1.0, &index);		// 4 bytes - mc_interface_get_rpm()
			data.inpVoltage 		= buffer_get_float16(message, 10.0, &index);		// 2 bytes - GET_INPUT_VOLTAGE()
			data.ampHours 			= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours(false)
			data.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours_charged(false)
			data.wattHours			= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours(false)
			data.wattHoursCharged	= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours_charged(false)
			data.tachometer 		= buffer_get_int32(message, &index);				// 4 bytes - mc_interface_get_tachometer_value(false)
			data.tachometerAbs 		= buffer_get_int32(message, &index);				// 4 bytes - mc_interface_get_tachometer_abs_value(false)
			data.error 				= (mc_fault_code)message[index++];								// 1 byte  - mc_interface_get_fault()
			data.pidPos				= buffer_get_float32(message, 1000000.0, &index);	// 4 bytes - mc_interface_get_pid_pos_now()
			data.id					= message[index++];								// 1 byte  - app_get_configuration()->controller_id	

			return true;

		break;

		/* case COMM_GET_VALUES_SELECTIVE:

			uint32_t mask = 0xFFFFFFFF; */

		default:
			return false;
		break;
	}
}

bool VescUart::getFWversion(void){
	return getFWversion(0);
}

bool VescUart::getFWversion(uint8_t canId){
	
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_FW_VERSION };

	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);
	if (messageLength > 0) { 
		return processReadPacket(message); 
	}
	return false;
}

bool VescUart::getVescValues(void) {
	return getVescValues(0);
}

bool VescUart::getVescValues(uint8_t canId) {

	if (debugPort!=NULL){
		debugPort->println("Command: COMM_GET_VALUES "+String(canId));
	}

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_GET_VALUES };

	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);

	if (messageLength > 55) {
		return processReadPacket(message); 
	}
	return false;
}
void VescUart::setNunchuckValues() {
	return setNunchuckValues(0);
}

void VescUart::setNunchuckValues(uint8_t canId) {

	if(debugPort!=NULL){
		debugPort->println("Command: COMM_SET_CHUCK_DATA "+String(canId));
	}	
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 11 : 13);
	uint8_t payload[payloadSize];

	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_CHUCK_DATA };
	payload[index++] = nunchuck.valueX;
	payload[index++] = nunchuck.valueY;
	buffer_append_bool(payload, nunchuck.lowerButton, &index);
	buffer_append_bool(payload, nunchuck.upperButton, &index);
	
	// Acceleration Data. Not used, Int16 (2 byte)
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;

	if(debugPort != NULL){
		debugPort->println("Nunchuck Values:");
		debugPort->print("x="); debugPort->print(nunchuck.valueX); debugPort->print(" y="); debugPort->print(nunchuck.valueY);
		debugPort->print(" LBTN="); debugPort->print(nunchuck.lowerButton); debugPort->print(" UBTN="); debugPort->println(nunchuck.upperButton);
	}

	packSendPayload(payload, payloadSize);
}

void VescUart::setCurrent(float current) {
	return setCurrent(current, 0);
}

void VescUart::setCurrent(float current, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_CURRENT };
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setBrakeCurrent(float brakeCurrent) {
	return setBrakeCurrent(brakeCurrent, 0);
}

void VescUart::setBrakeCurrent(float brakeCurrent, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_SET_CURRENT_BRAKE };
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUart::setRPM(float rpm) {
	return setRPM(rpm, 0);
}

void VescUart::setRPM(float rpm, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_RPM };
	buffer_append_int32(payload, (int32_t)(rpm), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setDuty(float duty) {
	return setDuty(duty, 0);
}

void VescUart::setDuty(float duty, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_DUTY };
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUart::sendKeepalive(void) {
	return sendKeepalive(0);
}

void VescUart::sendKeepalive(uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_ALIVE };
	packSendPayload(payload, payloadSize);
}

void VescUart::serialPrint(uint8_t * data, int len) {
	if(debugPort != NULL){
		for (int i = 0; i <= len; i++)
		{
			debugPort->print(data[i]);
			debugPort->print(" ");
		}
		debugPort->println("");
	}
}

void VescUart::printVescValues() {
	if(debugPort != NULL){
		debugPort->print("avgMotorCurrent: "); 	debugPort->println(data.avgMotorCurrent);
		debugPort->print("avgInputCurrent: "); 	debugPort->println(data.avgInputCurrent);
		debugPort->print("dutyCycleNow: "); 	debugPort->println(data.dutyCycleNow);
		debugPort->print("rpm: "); 				debugPort->println(data.rpm);
		debugPort->print("inputVoltage: "); 	debugPort->println(data.inpVoltage);
		debugPort->print("ampHours: "); 		debugPort->println(data.ampHours);
		debugPort->print("ampHoursCharged: "); 	debugPort->println(data.ampHoursCharged);
		debugPort->print("wattHours: "); 		debugPort->println(data.wattHours);
		debugPort->print("wattHoursCharged: "); debugPort->println(data.wattHoursCharged);
		debugPort->print("tachometer: "); 		debugPort->println(data.tachometer);
		debugPort->print("tachometerAbs: "); 	debugPort->println(data.tachometerAbs);
		debugPort->print("tempMosfet: "); 		debugPort->println(data.tempMosfet);
		debugPort->print("tempMotor: "); 		debugPort->println(data.tempMotor);
		debugPort->print("error: "); 			debugPort->println(data.error);
	}
}

void VescUart::sendFakeMcConf(void)
{
	mc_configuration mcconf;
	fillMcConfiguration(mcconf);
	uint8_t packet_buffer[512];
	packet_buffer[0] = COMM_SET_MCCONF;
	int32_t len = confgenerator_serialize_mcconf(packet_buffer + 1, &mcconf);
	packSendPayload(packet_buffer, len);
}

void VescUart::fillMcConfiguration(mc_configuration& config) {
    // Inizializza tutto a zero
    memset(&config, 0, sizeof(mc_configuration));
    
    // Limiti di corrente
    config.l_current_max = 60.0f;
    config.l_current_min = -60.0f;
    config.l_in_current_max = 30.0f;
    config.l_in_current_min = -30.0f;
    config.l_abs_current_max = 65.0f;
    
    // Limiti di tensione
    config.l_min_vin = 20.0f;
    config.l_max_vin = 50.4f;
    config.l_battery_cut_start = 42.0f;
    config.l_battery_cut_end = 40.0f;
    
    // RPM e duty cycle
    config.l_min_erpm = 100.0f;
    config.l_max_erpm = 100000.0f;
    config.l_min_duty = 0.05f;
    config.l_max_duty = 0.95f;
    
    // FOC settings
    config.foc_motor_r = 0.015f;
    config.foc_motor_l = 0.0002f;
    config.foc_motor_flux_linkage = 0.002f;
    config.foc_pll_kp = 0.004f;
    config.foc_pll_ki = 0.001f;
    config.foc_encoder_offset = 0.0f;
    config.foc_encoder_inverted = false;
    
    // PID velocità
    config.s_pid_kp = 0.01f;
    config.s_pid_ki = 0.02f;
    config.s_pid_kd = 0.0f;
    config.s_pid_min_erpm = 500.0f;
    config.s_pid_allow_braking = true;
    
    // PID posizione
    config.p_pid_kp = 0.1f;
    config.p_pid_ki = 0.02f;
    config.p_pid_kd = 0.005f;
    config.p_pid_ang_div = 180.0f;
    
    // Tipo di motore
    config.motor_type = MOTOR_TYPE_FOC;
    
    // Sensori
    config.sensor_mode = SENSOR_MODE_SENSORLESS;
    
    // Frequenza di commutazione
    config.m_bldc_f_sw_min = 20000.0f;
    config.m_bldc_f_sw_max = 40000.0f;
    
    // Numero di poli motore
    config.si_motor_poles = 14;
    
    // Dimensioni ruota e trasmissione
    config.si_gear_ratio = 4.5f;
    config.si_wheel_diameter = 0.65f;
    
    // CRC per sicurezza (valore fittizio)
    config.crc = 0xABCD;
}


int32_t VescUart::confgenerator_serialize_mcconf(uint8_t *buffer, const mc_configuration *conf) {
	int32_t ind = 0;

	buffer_append_uint32(buffer, MCCONF_SIGNATURE, &ind);

	buffer[ind++] = conf->pwm_mode;
	buffer[ind++] = conf->comm_mode;
	buffer[ind++] = conf->motor_type;
	buffer[ind++] = conf->sensor_mode;
	buffer_append_float32_auto(buffer, conf->l_current_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_current_min, &ind);
	buffer_append_float32_auto(buffer, conf->l_in_current_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_in_current_min, &ind);
	buffer_append_float16(buffer, 0.f, 10000, &ind);
	buffer_append_float16(buffer, 0.f, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->l_abs_current_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_min_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_erpm, &ind);
	buffer_append_float16(buffer, conf->l_erpm_start, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_erpm_fbrake, &ind);
	buffer_append_float32_auto(buffer, conf->l_max_erpm_fbrake_cc, &ind);
	buffer_append_float16(buffer, conf->l_min_vin, 10, &ind);
	buffer_append_float16(buffer, conf->l_max_vin, 10, &ind);
	buffer_append_float16(buffer, conf->l_battery_cut_start, 10, &ind);
	buffer_append_float16(buffer, conf->l_battery_cut_end, 10, &ind);
	buffer_append_float16(buffer, 0.f, 10, &ind);
	buffer_append_float16(buffer, 0.f, 10, &ind);
	buffer[ind++] = conf->l_slow_abs_current;
	buffer[ind++] = (uint8_t)conf->l_temp_fet_start;
	buffer[ind++] = (uint8_t)conf->l_temp_fet_end;
	buffer[ind++] = (uint8_t)conf->l_temp_motor_start;
	buffer[ind++] = (uint8_t)conf->l_temp_motor_end;
	buffer_append_float16(buffer, conf->l_temp_accel_dec, 10000, &ind);
	buffer_append_float16(buffer, conf->l_min_duty, 10000, &ind);
	buffer_append_float16(buffer, conf->l_max_duty, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->l_watt_max, &ind);
	buffer_append_float32_auto(buffer, conf->l_watt_min, &ind);
	buffer_append_float16(buffer, conf->l_current_max_scale, 10000, &ind);
	buffer_append_float16(buffer, conf->l_current_min_scale, 10000, &ind);
	buffer_append_float16(buffer, conf->l_duty_start, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->sl_min_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->sl_min_erpm_cycle_int_limit, &ind);
	buffer_append_float32_auto(buffer, conf->sl_max_fullbreak_current_dir_change, &ind);
	buffer_append_float16(buffer, conf->sl_cycle_int_limit, 10, &ind);
	buffer_append_float16(buffer, conf->sl_phase_advance_at_br, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->sl_cycle_int_rpm_br, &ind);
	buffer_append_float32_auto(buffer, conf->sl_bemf_coupling_k, &ind);
	buffer[ind++] = (uint8_t)conf->hall_table[0];
	buffer[ind++] = (uint8_t)conf->hall_table[1];
	buffer[ind++] = (uint8_t)conf->hall_table[2];
	buffer[ind++] = (uint8_t)conf->hall_table[3];
	buffer[ind++] = (uint8_t)conf->hall_table[4];
	buffer[ind++] = (uint8_t)conf->hall_table[5];
	buffer[ind++] = (uint8_t)conf->hall_table[6];
	buffer[ind++] = (uint8_t)conf->hall_table[7];
	buffer_append_float32_auto(buffer, conf->hall_sl_erpm, &ind);
	buffer_append_float32_auto(buffer, conf->foc_current_kp, &ind);
	buffer_append_float32_auto(buffer, conf->foc_current_ki, &ind);
	buffer_append_float32_auto(buffer, conf->foc_f_zv, &ind);
	buffer_append_float32_auto(buffer, conf->foc_dt_us, &ind);
	buffer[ind++] = conf->foc_encoder_inverted;
	buffer_append_float32_auto(buffer, conf->foc_encoder_offset, &ind);
	buffer_append_float32_auto(buffer, conf->foc_encoder_ratio, &ind);
	buffer[ind++] = conf->foc_sensor_mode;
	buffer_append_float32_auto(buffer, conf->foc_pll_kp, &ind);
	buffer_append_float32_auto(buffer, conf->foc_pll_ki, &ind);
	buffer_append_float32_auto(buffer, conf->foc_motor_l, &ind);
	buffer_append_float32_auto(buffer, conf->foc_motor_ld_lq_diff, &ind);
	buffer_append_float32_auto(buffer, conf->foc_motor_r, &ind);
	buffer_append_float32_auto(buffer, conf->foc_motor_flux_linkage, &ind);
	buffer_append_float32_auto(buffer, conf->foc_observer_gain, &ind);
	buffer_append_float32_auto(buffer, conf->foc_observer_gain_slow, &ind);
	buffer_append_float16(buffer, conf->foc_observer_offset, 1000, &ind);
	buffer_append_float32_auto(buffer, conf->foc_duty_dowmramp_kp, &ind);
	buffer_append_float32_auto(buffer, conf->foc_duty_dowmramp_ki, &ind);
	buffer_append_float16(buffer, 0.f, 10000, &ind);
	buffer_append_float32_auto(buffer, 0.f, &ind);
	buffer_append_float32_auto(buffer, conf->foc_openloop_rpm, &ind);
	buffer_append_float16(buffer, conf->foc_openloop_rpm_low, 1000, &ind);
	buffer_append_float16(buffer, conf->foc_d_gain_scale_start, 1000, &ind);
	buffer_append_float16(buffer, conf->foc_d_gain_scale_max_mod, 1000, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_hyst, 100, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_time_lock, 100, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_time_ramp, 100, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_time, 100, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_boost_q, 100, &ind);
	buffer_append_float16(buffer, conf->foc_sl_openloop_max_q, 100, &ind);
	buffer[ind++] = (uint8_t)conf->foc_hall_table[0];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[1];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[2];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[3];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[4];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[5];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[6];
	buffer[ind++] = (uint8_t)conf->foc_hall_table[7];
	buffer_append_float32_auto(buffer, conf->foc_hall_interp_erpm, &ind);
	buffer_append_float32_auto(buffer, 0.f, &ind);
	buffer_append_float32_auto(buffer, conf->foc_sl_erpm, &ind);
	buffer[ind++] = 0.f;
	buffer[ind++] = 0.f;
	buffer[ind++] = 0.f;
	buffer_append_float16(buffer, conf->foc_sat_comp, 1000, &ind);
	buffer[ind++] = conf->foc_temp_comp;
	buffer_append_float16(buffer, conf->foc_temp_comp_base_temp, 100, &ind);
	buffer_append_float16(buffer, conf->foc_current_filter_const, 10000, &ind);
	buffer[ind++] = conf->foc_cc_decoupling;
	buffer[ind++] = conf->foc_observer_type;
	buffer_append_float16(buffer, conf->foc_hfi_voltage_start, 10, &ind);
	buffer_append_float16(buffer, conf->foc_hfi_voltage_run, 10, &ind);
	buffer_append_float16(buffer, conf->foc_hfi_voltage_max, 10, &ind);
	buffer_append_float16(buffer, conf->foc_hfi_gain, 1000, &ind);
	buffer_append_float16(buffer, 0.f, 1000, &ind);
	buffer_append_float16(buffer, conf->foc_hfi_hyst, 100, &ind);
	buffer_append_float32_auto(buffer, conf->foc_sl_erpm_hfi, &ind);
	buffer_append_uint16(buffer, conf->foc_hfi_start_samples, &ind);
	buffer_append_float32_auto(buffer, conf->foc_hfi_obs_ovr_sec, &ind);
	buffer[ind++] = conf->foc_hfi_samples;
	buffer[ind++] = conf->foc_offsets_cal_on_boot;
	buffer_append_float32_auto(buffer, conf->foc_offsets_current[0], &ind);
	buffer_append_float32_auto(buffer, conf->foc_offsets_current[1], &ind);
	buffer_append_float32_auto(buffer, conf->foc_offsets_current[2], &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage[0], 10000, &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage[1], 10000, &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage[2], 10000, &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage_undriven[0], 10000, &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage_undriven[1], 10000, &ind);
	buffer_append_float16(buffer, conf->foc_offsets_voltage_undriven[2], 10000, &ind);
	buffer[ind++] = conf->foc_phase_filter_enable;
	buffer[ind++] = 0.f;
	buffer_append_float32_auto(buffer, conf->foc_phase_filter_max_erpm, &ind);
	buffer[ind++] = conf->foc_mtpa_mode;
	buffer_append_float32_auto(buffer, conf->foc_fw_current_max, &ind);
	buffer_append_float16(buffer, conf->foc_fw_duty_start, 10000, &ind);
	buffer_append_float16(buffer, conf->foc_fw_ramp_time, 1000, &ind);
	buffer_append_float16(buffer, conf->foc_fw_q_current_factor, 10000, &ind);
	buffer[ind++] = conf->foc_speed_soure;
	buffer[ind++] = 0.f;
	buffer[ind++] = conf->sp_pid_loop_rate;
	buffer_append_float32_auto(buffer, conf->s_pid_kp, &ind);
	buffer_append_float32_auto(buffer, conf->s_pid_ki, &ind);
	buffer_append_float32_auto(buffer, conf->s_pid_kd, &ind);
	buffer_append_float16(buffer, conf->s_pid_kd_filter, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->s_pid_min_erpm, &ind);
	buffer[ind++] = conf->s_pid_allow_braking;
	buffer_append_float32_auto(buffer, conf->s_pid_ramp_erpms_s, &ind);
	buffer[ind++] = 0.f;
	buffer_append_float32_auto(buffer, conf->p_pid_kp, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_ki, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_kd, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_kd_proc, &ind);
	buffer_append_float16(buffer, conf->p_pid_kd_filter, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_ang_div, &ind);
	buffer_append_float16(buffer, conf->p_pid_gain_dec_angle, 10, &ind);
	buffer_append_float32_auto(buffer, conf->p_pid_offset, &ind);
	buffer_append_float16(buffer, conf->cc_startup_boost_duty, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->cc_min_current, &ind);
	buffer_append_float32_auto(buffer, conf->cc_gain, &ind);
	buffer_append_float16(buffer, conf->cc_ramp_step_max, 10000, &ind);
	buffer_append_int32(buffer, conf->m_fault_stop_time_ms, &ind);
	buffer_append_float16(buffer, conf->m_duty_ramp_step, 10000, &ind);
	buffer_append_float32_auto(buffer, conf->m_current_backoff_gain, &ind);
	buffer_append_uint32(buffer, conf->m_encoder_counts, &ind);
	buffer_append_float16(buffer, 0.f, 1000, &ind);
	buffer_append_float16(buffer, 0.f, 1000, &ind);
	buffer_append_float16(buffer, 0.f, 1000, &ind);
	buffer_append_float16(buffer, 0.f, 1000, &ind);
	buffer_append_float16(buffer, 0.f, 1000, &ind);
	buffer_append_float16(buffer, 0.f, 1000, &ind);
	buffer[ind++] = conf->m_sensor_port_mode;
	buffer[ind++] = conf->m_invert_direction;
	buffer[ind++] = conf->m_drv8301_oc_mode;
	buffer[ind++] = (uint8_t)conf->m_drv8301_oc_adj;
	buffer_append_float32_auto(buffer, conf->m_bldc_f_sw_min, &ind);
	buffer_append_float32_auto(buffer, conf->m_bldc_f_sw_max, &ind);
	buffer_append_float32_auto(buffer, conf->m_dc_f_sw, &ind);
	buffer_append_float32_auto(buffer, conf->m_ntc_motor_beta, &ind);
	buffer[ind++] = conf->m_out_aux_mode;
	buffer[ind++] = conf->m_motor_temp_sens_type;
	buffer_append_float32_auto(buffer, conf->m_ptc_motor_coeff, &ind);
	buffer_append_float16(buffer, conf->m_ntcx_ptcx_res, 0.1, &ind);
	buffer_append_float16(buffer, conf->m_ntcx_ptcx_temp_base, 10, &ind);
	buffer[ind++] = (uint8_t)conf->m_hall_extra_samples;
	buffer[ind++] = (uint8_t)0.f;
	buffer[ind++] = (uint8_t)conf->si_motor_poles;
	buffer_append_float32_auto(buffer, conf->si_gear_ratio, &ind);
	buffer_append_float32_auto(buffer, conf->si_wheel_diameter, &ind);
	buffer[ind++] = conf->si_battery_type;
	buffer[ind++] = (uint8_t)conf->si_battery_cells;
	buffer_append_float32_auto(buffer, conf->si_battery_ah, &ind);
	buffer_append_float32_auto(buffer, conf->si_motor_nl_current, &ind);
	buffer[ind++] = conf->bms.type;
	buffer[ind++] = 0.f;
	buffer[ind++] = (uint8_t)conf->bms.t_limit_start;
	buffer[ind++] = (uint8_t)conf->bms.t_limit_end;
	buffer_append_float16(buffer, conf->bms.soc_limit_start, 1000, &ind);
	buffer_append_float16(buffer, conf->bms.soc_limit_end, 1000, &ind);
	buffer_append_float16(buffer, 0.f, 1000, &ind);
	buffer_append_float16(buffer, 0.f, 1000, &ind);
	buffer_append_float16(buffer, 0.f, 1000, &ind);
	buffer_append_float16(buffer, 0.f, 1000, &ind);
	buffer[ind++] = conf->bms.fwd_can_mode;

	return ind;
}
