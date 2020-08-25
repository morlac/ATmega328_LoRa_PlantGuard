/**
 * lora_config.hpp
 *
 *  Created on: Aug 23, 2020
 *      Author: adams
 */

#ifndef LORA_CONFIG_HPP_
#define LORA_CONFIG_HPP_

// LoRaWAN NwkSKey, network session key
#define _NWSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

// LoRaWAN AppSKey, application session key
#define _APPSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

// LoRaWAN end-device address (DevAddr)
#define _DEVADDR 0x00000000 // <-- Change this address for every node!

#define _ee_sensor_min { 500,  500,  500,  500}
#define _ee_sensor_max {2500, 2500, 2500, 2500}

#endif /* LORA_CONFIG_HPP_ */
