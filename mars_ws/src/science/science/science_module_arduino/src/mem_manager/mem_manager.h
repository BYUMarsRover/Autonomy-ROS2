#ifndef MEM_MANAGER_H
#define MEM_MANAGER_H

#include <EEPROM.h>
#include "mem_map.h"
#include "../definitions/definitions.h"
#include "../error/error.h"

#define EEPROM_DEBUG

template <typename T>
void EEPROM_writeObject(T* addr_eeprom, T& object) {
    EEPROM.put((size_t)addr_eeprom, object);

    #ifdef EEPROM_DEBUG
    Serial.print(F("Wrote EEPROM object "));
    Serial.print(object);
    Serial.print(F(" to addr "));
    Serial.println((size_t)addr_eeprom, HEX);
    #endif
}

template <typename T>
void EEPROM_writeArray(T* addr_eeprom, T* object_addr, uint16_t length) {
    for (int i = 0; i < length; i++) {
        EEPROM_writeObject(addr_eeprom + i, object_addr[i]);
    }
}

template <typename T>
void EEPROM_copyObject(T* addr_eeprom, T* destination) {
    T temp;
    *destination = EEPROM.get((size_t)addr_eeprom, temp);

    #ifdef EEPROM_DEBUG
    Serial.print(F("Copied EEPROM object from "));
    Serial.print(int(addr_eeprom));
    Serial.print(F(" to "));
    Serial.println(int(destination));
    #endif
}

template <typename T>
T EEPROM_readObject(T* addr_eeprom) {
    T temp;
    EEPROM.get((size_t)addr_eeprom, temp);

    #ifdef EEPROM_DEBUG
    Serial.print(F("Read EEPROM object "));
    Serial.print(temp);
    Serial.print(F(" from "));
    Serial.println(int(addr_eeprom));
    #endif

    return temp;
}

template <typename T>
void EEPROM_copyArray(T* addr_eeprom, T* destination, uint16_t length) {
    for (int i = 0; i < length; i++) {
        EEPROM_copyObject(addr_eeprom + i, destination + i);
    }
}

template <typename T>
void EEPROM_echoArray(T* addr_eeprom_cnt, T* addr_eeprom_array) {
    uint8_t cnt = EEPROM.read((size_t)addr_eeprom_cnt);
    response_buffer.error_code = (cnt == 0) ? ERROR_CODE_WARN : ERROR_CODE_SUCCESS;
    EEPROM_copyArray<T>(addr_eeprom_array, (T*)response_message_buffer, cnt);
    response_buffer.message_byte_len = cnt * sizeof(T);
}

template <typename T>
void EEPROM_pushSizedArray(T* addr_eeprom_cnt, T* addr_eeprom_array, T* source, uint8_t length) {
    EEPROM.write((size_t)addr_eeprom_cnt, length);
    EEPROM_copyArray<T>(source, addr_eeprom_array, length);
}

#endif /* MEM_MANAGER_H */