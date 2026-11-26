// based on: https://github.com/DomiStyle/esphome-panasonic-ac
#include "gree_ac_cnt.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include <cmath>
#include <cstring>
#include <cstdio>

namespace esphome {
namespace gree_ac {
namespace CNT {

static const char *const TAG = "gree_ac.serial";

static const uint8_t ALLOWED_PACKETS[] = {protocol::CMD_IN_UNIT_REPORT, protocol::CMD_IN_MODEL_ID};

void GreeACCNT::setup()
{
    GreeAC::setup();
    ESP_LOGD(TAG, "Using serial protocol for Gree AC");

    this->startup_special_sent_ = false;
    this->mac_packets_pending_ = 3;
    this->last_mac_sequence_millis_ = 0;
    this->last_sync_time_sent_ = millis() - 10000;
    this->last_packet_duration_ms_ = 0;
    /* allow immediate transmission of the first packet */
    this->last_packet_sent_ = millis() - protocol::TIME_REFRESH_PERIOD_MS - 1000;
}

void GreeACCNT::loop()
{
    /* this reads data from UART */
    GreeAC::loop();

    uint32_t now = millis();

    /* we have a frame from AC */
    if (this->serialProcess_.state == STATE_COMPLETE)
    {
        /* log for ESPHome debug */
        log_packet(this->serialProcess_.data, this->serialProcess_.size);

        /* mark that we have received a response (even if it might be invalid) */
        this->wait_response_ = false;

        if (verify_packet())  /* Verify length, header, counter and checksum */
        {
            this->last_packet_received_ = now;  /* Set the time at which we received our last packet */

            /* A valid recieved packet of accepted type marks module as being ready */
            if (this->state_ != ACState::Ready)
            {
                this->state_ = ACState::Ready;
                Component::status_clear_error();
            }

            handle_packet(); /* this will update state of components in HA as well as internal settings */
            yield();
        }

        /* restart for next packet */
        this->serialProcess_.size = 0;
        this->serialProcess_.state = STATE_WAIT_SYNC;
    }

    /* we will send a packet to the AC as a response to indicate changes */
    /* Check for 330ms gap since last packet finished transmission */
    if (now - this->last_packet_sent_ >= (protocol::TIME_REFRESH_PERIOD_MS + this->last_packet_duration_ms_))
    {
        if (!this->startup_special_sent_)
        {
            send_special_startup_packet();
        }
        else if (this->mac_packets_pending_ > 0)
        {
            send_mac_report_packet();
            this->mac_packets_pending_--;
            if (this->mac_packets_pending_ == 0)
            {
                this->last_mac_sequence_millis_ = now;
            }
        }
        else if (now - this->last_sync_time_sent_ >= 10000)
        {
            send_sync_time_packet();
        }
        else if (now - this->last_mac_sequence_millis_ >= protocol::TIME_MAC_CYCLE_PERIOD_MS)
        {
            this->mac_packets_pending_ = 6;
            /* next loop will start sending them */
        }
        else
        {
            send_params_set_packet();
        }
    }

    /* if there are no packets for some time - mark module as not ready */
    if (millis() - this->last_packet_received_ >= protocol::TIME_TIMEOUT_INACTIVE_MS)
    {
        if (this->state_ != ACState::Initializing)
        {
            this->state_ = ACState::Initializing;
            Component::status_set_error();
        }
    }
}

/*
 * ESPHome control request
 */

void GreeACCNT::mark_for_update_() {
    this->reqmodechange = true;
    this->update_ = ACUpdate::UpdateStart;
}

void GreeACCNT::control(const climate::ClimateCall &call)
{
    if (this->state_ != ACState::Ready)
        return;

    if (call.get_mode().has_value())
    {
        ESP_LOGV(TAG, "Requested mode change");
        this->mark_for_update_();
        this->mode = *call.get_mode();

        if (this->light_mode_ == light_options::AUTO)
        {
            this->light_state_ = (this->mode != climate::CLIMATE_MODE_OFF);
        }
    }

    if (call.get_target_temperature().has_value())
    {
        ESP_LOGV(TAG, "Requested target teperature change");
        this->mark_for_update_();
        this->target_temperature = *call.get_target_temperature();
        if (this->target_temperature < MIN_TEMPERATURE)
        {
            this->target_temperature = MIN_TEMPERATURE;
        }
        else if (this->target_temperature > MAX_TEMPERATURE)
        {
            this->target_temperature = MAX_TEMPERATURE;
        }
    }

    if (call.get_fan_mode().has_value())
    {
        ESP_LOGV(TAG, "Requested fan mode change");
        this->mark_for_update_();
        this->fan_mode = *call.get_fan_mode();
        this->clear_custom_fan_mode_();

        /* Requirement 3: When the fan mode gets changed while turbo is on, the turbo mode must be deactivated.
           Also for quiet mode. */
        this->update_turbo(false);
        this->update_quiet(quiet_options::OFF);
    }

    if (call.has_custom_fan_mode())
    {
        ESP_LOGV(TAG, "Requested custom fan mode change");
        this->mark_for_update_();
        this->set_custom_fan_mode_(call.get_custom_fan_mode());
        this->fan_mode.reset();

        /* Requirement 3: When the fan mode gets changed while turbo is on, the turbo mode must be deactivated.
           Also for quiet mode. */
        this->update_turbo(false);
        this->update_quiet(quiet_options::OFF);
    }

    if (call.get_swing_mode().has_value())
    {
        ESP_LOGV(TAG, "Requested swing mode change");
        this->mark_for_update_();
        switch (*call.get_swing_mode()) {
            case climate::CLIMATE_SWING_BOTH:
                this->vertical_swing_state_   =   vertical_swing_options::FULL;
                this->horizontal_swing_state_ = horizontal_swing_options::FULL;
                break;
            case climate::CLIMATE_SWING_OFF:
                /* both center */
                this->vertical_swing_state_   =   vertical_swing_options::CMID;
                this->horizontal_swing_state_ = horizontal_swing_options::CMID;
                break;
            case climate::CLIMATE_SWING_VERTICAL:
                /* vertical full, horizontal center */
                this->vertical_swing_state_   =   vertical_swing_options::FULL;
                this->horizontal_swing_state_ = horizontal_swing_options::CMID;
                break;
            case climate::CLIMATE_SWING_HORIZONTAL:
                /* horizontal full, vertical center */
                this->vertical_swing_state_   =   vertical_swing_options::CMID;
                this->horizontal_swing_state_ = horizontal_swing_options::FULL;
                break;
            default:
                ESP_LOGV(TAG, "Unsupported swing mode requested");
                /* both center */
                this->vertical_swing_state_   =   vertical_swing_options::CMID;
                this->horizontal_swing_state_ = horizontal_swing_options::CMID;
                break;
        }
    }
}

void GreeACCNT::transmit_packet(const uint8_t *packet, size_t length)
{
    this->last_packet_sent_ = millis();
    this->last_packet_duration_ms_ = (length * 11000) / 4800;

    log_packet(packet, length, true);

    if (this->enable_tx_switch_ == nullptr || this->enable_tx_switch_->state) {
        write_array(packet, length);
    }
    yield();
}

void GreeACCNT::send_special_startup_packet()
{
    uint8_t packet[] = {
        0x7E, 0x7E, 0x10, 0x02,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
        0x28, 0x1E, 0x19, 0x23, 0x23, 0x00, 0xBA
    };
    transmit_packet(packet, sizeof(packet));
    this->startup_special_sent_ = true;
    ESP_LOGD(TAG, "Sent special startup packet");
}

/*
 * Send a raw packet, as is
 */
void GreeACCNT::send_params_set_packet()
{
    if (this->wait_response_)
    {
        if (millis() - this->last_packet_sent_ < protocol::TIME_WAIT_RESPONSE_TIMEOUT_MS)
        {
            /* waiting for report to come */
            return;
        }
        else
        {
            ESP_LOGW(TAG, "Timed out waiting for response from AC unit");
            this->wait_response_ = false;
        }
    }

    uint8_t payload[protocol::SET_PACKET_LEN];
    memset(payload, 0, sizeof(payload));
    
    payload[protocol::SET_CONST_02_BYTE] = protocol::SET_CONST_02_VAL; /* Some always 0x02 byte... */
    payload[protocol::SET_CONST_BIT_BYTE] = protocol::SET_CONST_BIT_MASK; /* Some always true bit */

    /* Prepare the rest of the frame */
    /* this handles tricky part of 0xAF value and flag marking that WiFi does not apply any changes */
    switch(this->update_)
    {
        default:
        case ACUpdate::NoUpdate:
            // payload[protocol::SET_NOCHANGE_BYTE] |= protocol::SET_NOCHANGE_MASK; // Bit 0x08 at byte 11 indicates no change
            break;
        case ACUpdate::UpdateStart:
            payload[protocol::SET_AF_BYTE] = protocol::SET_AF_VAL;
            break;
        case ACUpdate::UpdateClear:
            break;
    }

    /* MODE and POWER --------------------------------------------------------------------------- */
    uint8_t mode = protocol::REPORT_MODE_AUTO;
    bool power = false;
    switch (this->mode)
    {
        case climate::CLIMATE_MODE_AUTO:
            mode = protocol::REPORT_MODE_AUTO;
            power = true;
            break;
        case climate::CLIMATE_MODE_COOL:
            mode = protocol::REPORT_MODE_COOL;
            power = true;
            break;
        case climate::CLIMATE_MODE_DRY:
            mode = protocol::REPORT_MODE_DRY;
            power = true;
            break;
        case climate::CLIMATE_MODE_FAN_ONLY:
            mode = protocol::REPORT_MODE_FAN;
            power = true;
            break;
        case climate::CLIMATE_MODE_HEAT:
            mode = protocol::REPORT_MODE_HEAT;
            power = true;
            break;
        default:
        case climate::CLIMATE_MODE_OFF:
            /* In case of MODE_OFF we will not alter the last mode setting recieved from AC, see determine_mode() */
            switch (this->mode_internal_)
            {
                case climate::CLIMATE_MODE_AUTO:
                    mode = protocol::REPORT_MODE_AUTO;
                    break;
                case climate::CLIMATE_MODE_COOL:
                    mode = protocol::REPORT_MODE_COOL;
                    break;
                case climate::CLIMATE_MODE_DRY:
                    mode = protocol::REPORT_MODE_DRY;
                    break;
                case climate::CLIMATE_MODE_FAN_ONLY:
                    mode = protocol::REPORT_MODE_FAN;
                    break;
                case climate::CLIMATE_MODE_HEAT:
                    mode = protocol::REPORT_MODE_HEAT;
                    break;
                default:
                    break;
            }
            power = false;
            break;
    }

    payload[protocol::REPORT_MODE_BYTE] |= (mode << protocol::REPORT_MODE_POS);
    if (power)
    {
        payload[protocol::REPORT_PWR_BYTE] |= protocol::REPORT_PWR_MASK;
    }

    // TARGET TEMPERATURE
    uint8_t target_temperature = static_cast<uint8_t>(round(this->target_temperature));
    payload[protocol::REPORT_TEMP_SET_BYTE] |= ((target_temperature - protocol::REPORT_TEMP_SET_OFF) << protocol::REPORT_TEMP_SET_POS) & protocol::REPORT_TEMP_SET_MASK;

    // FAN STATE
    uint8_t fan_mode_payload4 = 0x00;
    uint8_t fan_mode_payload18 = 0x00;

    if (this->fan_mode.has_value()) {
        switch (*this->fan_mode) {
            case climate::CLIMATE_FAN_AUTO:
                fan_mode_payload4 = 0x00;
                fan_mode_payload18 = 0x00;
                break;
            case climate::CLIMATE_FAN_LOW:
                fan_mode_payload4 = 0x02;
                fan_mode_payload18 = 0x02;
                break;
            case climate::CLIMATE_FAN_MEDIUM:
                fan_mode_payload4 = 0x02;
                fan_mode_payload18 = 0x03;
                break;
            case climate::CLIMATE_FAN_HIGH:
                fan_mode_payload4 = 0x03;
                fan_mode_payload18 = 0x04;
                break;
            default:
                break;
        }
    }

    if (this->has_custom_fan_mode()) {
        const auto custom_fan_mode = this->get_custom_fan_mode();

        if (custom_fan_mode == fan_modes::FAN_MIN) {
            fan_mode_payload4 = 0x01;
            fan_mode_payload18 = 0x01;
        } else if (custom_fan_mode == fan_modes::FAN_MAX) {
            fan_mode_payload4 = 0x03;
            fan_mode_payload18 = 0x05;
        }
    }

    payload[protocol::REPORT_FAN_SPD1_BYTE] = 0;
    payload[protocol::REPORT_FAN_SPD1_BYTE] |= (fan_mode_payload18 & protocol::REPORT_FAN_SPD1_MASK);

    // Clear the bits for fan speed 2 and set them according to the desired fan mode
    payload[protocol::REPORT_FAN_SPD2_BYTE] &= ~protocol::REPORT_FAN_SPD2_MASK;
    payload[protocol::REPORT_FAN_SPD2_BYTE] |= (fan_mode_payload4 & protocol::REPORT_FAN_SPD2_MASK);

    // TURBO STATE
    if (this->turbo_state_)
    {
        payload[protocol::REPORT_FAN_TURBO_BYTE] |= protocol::REPORT_FAN_TURBO_MASK;
    }

    // QUIET STATE
    if (this->quiet_state_ == quiet_options::ON)
    {
        payload[protocol::REPORT_FAN_QUIET_BYTE] |= protocol::REPORT_FAN_QUIET_MASK;
    }
    else if (this->quiet_state_ == quiet_options::AUTO)
    {
        payload[protocol::REPORT_FAN_QUIET_BYTE] |= protocol::REPORT_FAN_QUIET_AUTO_MASK;
    }

    // VERTICAL SWING
    static const struct { const char* const opt; uint8_t val; } VSWING_MAP[] = {
        {vertical_swing_options::OFF,   protocol::REPORT_VSWING_OFF},
        {vertical_swing_options::FULL,  protocol::REPORT_VSWING_FULL},
        {vertical_swing_options::DOWN,  protocol::REPORT_VSWING_DOWN},
        {vertical_swing_options::MIDD,  protocol::REPORT_VSWING_MIDD},
        {vertical_swing_options::MID,   protocol::REPORT_VSWING_MID},
        {vertical_swing_options::MIDU,  protocol::REPORT_VSWING_MIDU},
        {vertical_swing_options::UP,    protocol::REPORT_VSWING_UP},
        {vertical_swing_options::CDOWN, protocol::REPORT_VSWING_CDOWN},
        {vertical_swing_options::CMIDD, protocol::REPORT_VSWING_CMIDD},
        {vertical_swing_options::CMID,  protocol::REPORT_VSWING_CMID},
        {vertical_swing_options::CMIDU, protocol::REPORT_VSWING_CMIDU},
        {vertical_swing_options::CUP,   protocol::REPORT_VSWING_CUP},
    };

    uint8_t mode_vertical_swing = protocol::REPORT_VSWING_OFF;
    for (const auto& mapping : VSWING_MAP) {
        if (this->vertical_swing_state_ == mapping.opt) {
            mode_vertical_swing = mapping.val;
            break;
        }
    }

    payload[protocol::REPORT_VSWING_BYTE] |= (mode_vertical_swing << protocol::REPORT_VSWING_POS);

    // HORIZONTAL SWING
    static const struct { const char* const opt; uint8_t val; } HSWING_MAP[] = {
        {horizontal_swing_options::OFF,    protocol::REPORT_HSWING_OFF},
        {horizontal_swing_options::FULL,   protocol::REPORT_HSWING_FULL},
        {horizontal_swing_options::CLEFT,  protocol::REPORT_HSWING_CLEFT},
        {horizontal_swing_options::CMIDL,  protocol::REPORT_HSWING_CMIDL},
        {horizontal_swing_options::CMID,   protocol::REPORT_HSWING_CMID},
        {horizontal_swing_options::CMIDR,  protocol::REPORT_HSWING_CMIDR},
        {horizontal_swing_options::CRIGHT, protocol::REPORT_HSWING_CRIGHT},
    };

    uint8_t mode_horizontal_swing = protocol::REPORT_HSWING_OFF;
    for (const auto& mapping : HSWING_MAP) {
        if (this->horizontal_swing_state_ == mapping.opt) {
            mode_horizontal_swing = mapping.val;
            break;
        }
    }
    payload[protocol::REPORT_HSWING_BYTE] |= (mode_horizontal_swing << protocol::REPORT_HSWING_POS);

    /* DISPLAY --------------------------------------------------------------------------- */
    uint8_t display_mode = protocol::REPORT_DISP_MODE_SET;
    if (this->mode != climate::CLIMATE_MODE_OFF)
    {
        if (this->display_state_ == display_options::SET)
        {
            display_mode = protocol::REPORT_DISP_MODE_SET;
        }
        else if (this->display_state_ == display_options::ACT)
        {
            display_mode = protocol::REPORT_DISP_MODE_ACT;
        }
    }
    else
    {
        // When the AC is off always sent "set" no matter what the user selected.
        display_mode = protocol::REPORT_DISP_MODE_SET;
    }

    payload[protocol::REPORT_DISP_MODE_BYTE] |= (display_mode << protocol::REPORT_DISP_MODE_POS);

    if (this->light_state_)
    {
        payload[protocol::REPORT_DISP_ON_BYTE] |= protocol::REPORT_DISP_ON_MASK;
    }

    /* DISPLAY UNIT --------------------------------------------------------------------------- */
    if (this->display_unit_state_ == display_unit_options::DEGF)
    {
        payload[protocol::REPORT_DISP_F_BYTE] |= protocol::REPORT_DISP_F_MASK;
    }

    /* IONIZER -------------------------------------------------------------------------- */
    if (this->ionizer_state_)
    {
        payload[protocol::REPORT_IONIZER1_BYTE] |= protocol::REPORT_IONIZER1_MASK;
        payload[protocol::REPORT_IONIZER2_BYTE] |= protocol::REPORT_IONIZER2_MASK;
    }

    /* BEEPER --------------------------------------------------------------------------- */
    if (!this->beeper_state_)
    {
        payload[protocol::REPORT_BEEPER_BYTE] |= protocol::REPORT_BEEPER_MASK;
    }

    /* SLEEP --------------------------------------------------------------------------- */
    if (this->sleep_state_)
    {
        payload[protocol::REPORT_SLEEP_BYTE] |= protocol::REPORT_SLEEP_MASK;
    }

    /* XFAN --------------------------------------------------------------------------- */
    if (this->xfan_state_)
    {
        payload[protocol::REPORT_XFAN_BYTE] |= protocol::REPORT_XFAN_MASK;
    }

    /* POWERSAVE --------------------------------------------------------------------------- */
    if (this->powersave_state_)
    {
        payload[protocol::REPORT_POWERSAVE_BYTE] |= protocol::REPORT_POWERSAVE_MASK;
    }

    /* IFEEL --------------------------------------------------------------------------- */
    if (this->ifeel_state_)
    {
        payload[protocol::REPORT_IFEEL_BYTE] |= protocol::REPORT_IFEEL_MASK;
    }

    /* Do the command, length */

    uint8_t full_packet[protocol::SET_PACKET_LEN + 5];
    full_packet[0] = protocol::SYNC;
    full_packet[1] = protocol::SYNC;
    full_packet[2] = protocol::SET_PACKET_LEN + 2;
    full_packet[3] = protocol::CMD_OUT_PARAMS_SET;
    memcpy(&full_packet[4], payload, protocol::SET_PACKET_LEN);

    finalize_checksum_(full_packet, sizeof(full_packet));

    this->wait_response_ = true;
    transmit_packet(full_packet, sizeof(full_packet));

    /* update setting state-machine */
    switch(this->update_)
    {
        case ACUpdate::NoUpdate:
            break;
        case ACUpdate::UpdateStart:
            this->update_ = ACUpdate::NoUpdate; // Transition directly to NoUpdate to send AF only once
            break;
        case ACUpdate::UpdateClear:
            this->update_ = ACUpdate::NoUpdate;
            break;
        default:
            this->update_ = ACUpdate::NoUpdate;
            break;
    }
}

void GreeACCNT::send_mac_report_packet()
{
    uint8_t full_packet[16];
    uint8_t mac[6];
    get_mac_address_raw(mac);

    full_packet[0] = protocol::SYNC;
    full_packet[1] = protocol::SYNC;
    full_packet[2] = 0x0D; // Length
    full_packet[3] = protocol::CMD_OUT_MAC_REPORT;
    full_packet[4] = 0x07;
    full_packet[5] = 0x00;
    full_packet[6] = 0x00;
    full_packet[7] = 0x00;
    memcpy(&full_packet[8], mac, 6);
    full_packet[14] = 0x00;

    finalize_checksum_(full_packet, sizeof(full_packet));

    ESP_LOGD(TAG, "Sending MAC report: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    transmit_packet(full_packet, 16);
}

void GreeACCNT::send_sync_time_packet()
{
    uint8_t full_packet[17];
    full_packet[0] = protocol::SYNC;
    full_packet[1] = protocol::SYNC;
    full_packet[2] = 0x0E; // Length
    full_packet[3] = protocol::CMD_OUT_SYNC_TIME;
    full_packet[4] = 0x04;
    memset(&full_packet[5], 0, 10);
    full_packet[15] = 0x7E;

    finalize_checksum_(full_packet, sizeof(full_packet));

    ESP_LOGD(TAG, "Sending sync time packet");
    transmit_packet(full_packet, 17);
    this->last_sync_time_sent_ = millis();
}

/*
 * Packet handling
 */

uint8_t GreeACCNT::calculate_checksum_(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 2; i < len - 1; i++) {
        checksum += data[i];
    }
    return checksum;
}

void GreeACCNT::finalize_checksum_(uint8_t *data, size_t len) {
    data[len - 1] = calculate_checksum_(data, len);
}

bool GreeACCNT::verify_checksum_(const uint8_t *data, size_t len) {
    return data[len - 1] == calculate_checksum_(data, len);
}

bool GreeACCNT::verify_packet()
{
    /* At least 2 sync bytes + length + type + checksum */
    if (this->serialProcess_.size < 5)
    {
        ESP_LOGW(TAG, "Dropping invalid packet (length)");
        return false;
    }

    /* The header (aka sync bytes) was checked by GreeAC::loop() */

    /* The frame len was assumed by GreeAC::loop() */

    /* Check if this packet type sould be processed */
    bool commandAllowed = false;
    for (uint8_t packet : ALLOWED_PACKETS)
    {
        if (this->serialProcess_.data[3] == packet)
        {
            commandAllowed = true;
            break;
        }
    }
    if (!commandAllowed)
    {
        ESP_LOGW(TAG, "Dropping invalid packet (command [%02X] not allowed)", this->serialProcess_.data[3]);
        return false;
    }

    if (!verify_checksum_(this->serialProcess_.data, this->serialProcess_.size))
    {
        ESP_LOGD(TAG, "Dropping invalid packet (checksum)");
        return false;
    }

    return true;
}

void GreeACCNT::handle_packet()
{
    if (this->serialProcess_.data[3] == protocol::CMD_IN_UNIT_REPORT)
    {
        if (this->update_ != ACUpdate::NoUpdate) {
            return;
        }

        /* Move payload to front of data array to simplify indexing (remove 4 byte header) */
        size_t payload_size = this->serialProcess_.size - 5;
        memmove(this->serialProcess_.data, &this->serialProcess_.data[4], payload_size);
        this->serialProcess_.size = payload_size;

        /* now process the data */
        bool hasChanged = this->processUnitReport();

        if (hasChanged || reqmodechange)
        {
            ESP_LOGD(TAG, "State update: hasChanged=%d, reqmodechange=%d", hasChanged, reqmodechange);
            this->publish_state();
            reqmodechange = false;
        }
    }
    else if (this->serialProcess_.data[3] == protocol::CMD_IN_MODEL_ID)
    {
        if (this->serialProcess_.size < 7) {
            ESP_LOGW(TAG, "Model ID packet too short");
            return;
        }
        uint8_t b1 = this->serialProcess_.data[4];
        uint8_t b2 = this->serialProcess_.data[5];
        uint8_t b3 = this->serialProcess_.data[6];

        char buf[32];
        snprintf(buf, sizeof(buf), "%d%02d%02d", b1, b2, b3);
        std::string model_id = buf;

        ESP_LOGI(TAG, "Received Model ID: %s", model_id.c_str());

        if (this->model_id_text_sensor_ != nullptr && this->model_id_text_sensor_->state != model_id) {
            this->model_id_text_sensor_->publish_state(model_id);
        }
    }
    else
    {
        ESP_LOGD(TAG, "Received unknown packet type: 0x%02X", this->serialProcess_.data[3]);
    }
}

/*
 * This decodes frame recieved from AC Unit
 */
bool GreeACCNT::processUnitReport()
{
    bool hasChanged = false;
    bool modeChanged = false;

    climate::ClimateMode newMode = determine_mode();
    if (this->mode != newMode) {
        this->mode = newMode;
        hasChanged = true;
        modeChanged = true;

        if (this->light_mode_ == light_options::AUTO)
        {
            bool new_light_state = (this->mode != climate::CLIMATE_MODE_OFF);
            if (this->light_state_ != new_light_state) {
                this->light_state_ = new_light_state;
                this->update_ = ACUpdate::UpdateStart;
            }
        }
    }
   
    uint8_t temset = (this->serialProcess_.data[protocol::REPORT_TEMP_SET_BYTE] & protocol::REPORT_TEMP_SET_MASK) >> protocol::REPORT_TEMP_SET_POS;
    hasChanged |= this->update_target_temperature((float)(temset + protocol::REPORT_TEMP_SET_OFF));
    hasChanged |= this->update_current_temperature((float)(this->serialProcess_.data[protocol::REPORT_TEMP_ACT_BYTE] - protocol::REPORT_TEMP_ACT_OFF));

    const char* verticalSwing = determine_vertical_swing();
    hasChanged |= this->update_swing_vertical(verticalSwing);

    const char* horizontalSwing = determine_horizontal_swing();
    hasChanged |= this->update_swing_horizontal(horizontalSwing);

    climate::ClimateSwingMode newSwingMode;
    if (strcmp(verticalSwing, vertical_swing_options::FULL) == 0 && strcmp(horizontalSwing, horizontal_swing_options::FULL) == 0)
        newSwingMode = climate::CLIMATE_SWING_BOTH;
    else if (strcmp(verticalSwing, vertical_swing_options::FULL) == 0)
        newSwingMode = climate::CLIMATE_SWING_VERTICAL;
    else if (strcmp(horizontalSwing, horizontal_swing_options::FULL) == 0)
        newSwingMode = climate::CLIMATE_SWING_HORIZONTAL;
    else
        newSwingMode = climate::CLIMATE_SWING_OFF;
    
    if (this->swing_mode != newSwingMode) {
        this->swing_mode = newSwingMode;
        hasChanged = true;
    }

    const char* display = determine_display();
    if (this->mode != climate::CLIMATE_MODE_OFF) {
        if (modeChanged && this->display_state_ == display_options::ACT) {
            // Unit just turned ON and we want Actual temperature.
            // Force an update to re-apply the setting.
            this->update_ = ACUpdate::UpdateStart;
        }
        hasChanged |= this->update_display(display);
    } else {
        // When OFF, AC unit always reports "Set temperature".
        // We only follow it if it's "Actual" (unlikely when OFF) or if we don't have a state yet.
        if (this->display_state_.empty() || strcmp(display, display_options::ACT) == 0) {
            hasChanged |= this->update_display(display);
        }
    }

    bool light_reported = determine_light();
    if (this->light_state_ != light_reported || (this->light_select_ != nullptr && this->light_select_->current_option().empty())) {
        if (this->light_mode_ == light_options::AUTO)
        {
            if (!modeChanged) {
                // Remote override: AC power state did not change, but light status changed.
                // We accept the new status as our desired state.
                this->light_state_ = light_reported;
            }
            // else: Mode changed, we keep our calculated light_state_ and UpdateStart set above.

            hasChanged |= this->update_light(this->light_state_);
        }
        else if (this->light_mode_ == light_options::OFF)
        {
            if (light_reported == true) {
                // Enforce OFF: unit reported light ON, so we force it back to OFF.
                this->update_ = ACUpdate::UpdateStart;
                this->light_state_ = false;
            } else {
                hasChanged |= this->update_light(false);
            }
        }
        else if (this->light_mode_ == light_options::ON)
        {
            if (light_reported == false) {
                // Enforce ON: unit reported light OFF, so we force it back to ON.
                this->update_ = ACUpdate::UpdateStart;
                this->light_state_ = true;
            } else {
                hasChanged |= this->update_light(true);
            }
        }
    }

    hasChanged |= this->update_display_unit(determine_display_unit());
    hasChanged |= this->update_ionizer(determine_ionizer());
    hasChanged |= this->update_beeper(determine_beeper());
    hasChanged |= this->update_sleep(determine_sleep());
    hasChanged |= this->update_xfan(determine_xfan());
    hasChanged |= this->update_powersave(determine_powersave());
    hasChanged |= this->update_turbo(determine_turbo());
    hasChanged |= this->update_ifeel(determine_ifeel());
    hasChanged |= this->update_quiet(determine_quiet());
    hasChanged |= this->update_fan_mode(determine_fan_mode());

    return hasChanged;
}

climate::ClimateMode GreeACCNT::determine_mode()
{
    uint8_t mode = (this->serialProcess_.data[protocol::REPORT_MODE_BYTE] & protocol::REPORT_MODE_MASK) >> protocol::REPORT_MODE_POS;

    /* as mode presented by climate component incorporates both power and mode we will store this separately for Gree
       in _internal_ fields */
    /* check unit power flag */
    this->power_internal_ = (this->serialProcess_.data[protocol::REPORT_PWR_BYTE] & protocol::REPORT_PWR_MASK) != 0;

    /* check unit mode */
    switch (mode)
    {
        case protocol::REPORT_MODE_AUTO:
            this->mode_internal_ = climate::CLIMATE_MODE_AUTO;
            break;
        case protocol::REPORT_MODE_COOL:
            this->mode_internal_ = climate::CLIMATE_MODE_COOL;
            break;
        case protocol::REPORT_MODE_DRY:
            this->mode_internal_ = climate::CLIMATE_MODE_DRY;
            break;
        case protocol::REPORT_MODE_FAN:
            this->mode_internal_ = climate::CLIMATE_MODE_FAN_ONLY;
            break;
        case protocol::REPORT_MODE_HEAT:
            this->mode_internal_ = climate::CLIMATE_MODE_HEAT;
            break;
        default:
            ESP_LOGW(TAG, "Received unknown climate mode");
            this->mode_internal_ = climate::CLIMATE_MODE_OFF;
            break;
    }

    /* if unit is powered on - return the mode, otherwise return CLIMATE_MODE_OFF */
    if (this->power_internal_)
    {
        return this->mode_internal_;
    }
    else
    {
        return climate::CLIMATE_MODE_OFF;
    }
}

const char* GreeACCNT::determine_fan_mode()
{
    /* fan setting has quite complex representation in the packet, brace for it */
    uint8_t fan_mode = (this->serialProcess_.data[protocol::REPORT_FAN_SPD1_BYTE] & protocol::REPORT_FAN_SPD1_MASK);

    switch (fan_mode) {
        case 0x01:
            return fan_modes::FAN_MIN;
        case 0x02:
            return fan_modes::FAN_LOW;
        case 0x03:
            return fan_modes::FAN_MED;
        case 0x04:
            return fan_modes::FAN_HIGH;
        case 0x05:
            return fan_modes::FAN_MAX;
        case 0x00:
            return fan_modes::FAN_AUTO;
        default:
            ESP_LOGW(TAG, "Received unknown fan mode: %d", fan_mode);
            return fan_modes::FAN_AUTO;
    }
}

const char* GreeACCNT::determine_vertical_swing()
{
    uint8_t mode = (this->serialProcess_.data[protocol::REPORT_VSWING_BYTE]  & protocol::REPORT_VSWING_MASK) >> protocol::REPORT_VSWING_POS;

    static const struct { uint8_t val; const char* const opt; } VSWING_MAP[] = {
        {protocol::REPORT_VSWING_OFF,   vertical_swing_options::OFF},
        {protocol::REPORT_VSWING_FULL,  vertical_swing_options::FULL},
        {protocol::REPORT_VSWING_CUP,   vertical_swing_options::CUP},
        {protocol::REPORT_VSWING_CMIDU, vertical_swing_options::CMIDU},
        {protocol::REPORT_VSWING_CMID,  vertical_swing_options::CMID},
        {protocol::REPORT_VSWING_CMIDD, vertical_swing_options::CMIDD},
        {protocol::REPORT_VSWING_CDOWN, vertical_swing_options::CDOWN},
        {protocol::REPORT_VSWING_DOWN,  vertical_swing_options::DOWN},
        {protocol::REPORT_VSWING_MIDD,  vertical_swing_options::MIDD},
        {protocol::REPORT_VSWING_MID,   vertical_swing_options::MID},
        {protocol::REPORT_VSWING_MIDU,  vertical_swing_options::MIDU},
        {protocol::REPORT_VSWING_UP,    vertical_swing_options::UP},
    };

    for (const auto& mapping : VSWING_MAP) {
        if (mode == mapping.val) return mapping.opt;
    }

    ESP_LOGW(TAG, "Received unknown vertical swing mode");
    return vertical_swing_options::OFF;
}

const char* GreeACCNT::determine_horizontal_swing()
{
    uint8_t mode = (this->serialProcess_.data[protocol::REPORT_HSWING_BYTE]  & protocol::REPORT_HSWING_MASK) >> protocol::REPORT_HSWING_POS;

    static const struct { uint8_t val; const char* const opt; } HSWING_MAP[] = {
        {protocol::REPORT_HSWING_OFF,    horizontal_swing_options::OFF},
        {protocol::REPORT_HSWING_FULL,   horizontal_swing_options::FULL},
        {protocol::REPORT_HSWING_CLEFT,  horizontal_swing_options::CLEFT},
        {protocol::REPORT_HSWING_CMIDL,  horizontal_swing_options::CMIDL},
        {protocol::REPORT_HSWING_CMID,   horizontal_swing_options::CMID},
        {protocol::REPORT_HSWING_CMIDR,  horizontal_swing_options::CMIDR},
        {protocol::REPORT_HSWING_CRIGHT, horizontal_swing_options::CRIGHT},
    };

    for (const auto& mapping : HSWING_MAP) {
        if (mode == mapping.val) return mapping.opt;
    }

    ESP_LOGW(TAG, "Received unknown horizontal swing mode");
    return horizontal_swing_options::OFF;
}

const char* GreeACCNT::determine_display()
{
    uint8_t mode = (this->serialProcess_.data[protocol::REPORT_DISP_MODE_BYTE] & protocol::REPORT_DISP_MODE_MASK) >> protocol::REPORT_DISP_MODE_POS;

    switch (mode) {
        case protocol::REPORT_DISP_MODE_AUTO:
            return display_options::SET;
        case protocol::REPORT_DISP_MODE_SET:
            return display_options::SET;
        case protocol::REPORT_DISP_MODE_ACT:
            return display_options::ACT;
        case protocol::REPORT_DISP_MODE_OUT:
            ESP_LOGW(TAG, "Outside temperature display mode is not supported and was requested by the unit. Falling back to Set temperature.");
            return display_options::SET;
        default:
            ESP_LOGW(TAG, "Received unknown display mode: %d. Falling back to Set temperature.", mode);
            return display_options::SET;
    }
}

bool GreeACCNT::determine_light()
{
    return (this->serialProcess_.data[protocol::REPORT_DISP_ON_BYTE] & protocol::REPORT_DISP_ON_MASK) != 0;
}

const char* GreeACCNT::determine_display_unit()
{
    if (this->serialProcess_.data[protocol::REPORT_DISP_F_BYTE] & protocol::REPORT_DISP_F_MASK)
    {
        return display_unit_options::DEGF;
    }
    else
    {
        return display_unit_options::DEGC;
    }
}

bool GreeACCNT::determine_ionizer(){
    bool ionizer1 = (this->serialProcess_.data[protocol::REPORT_IONIZER1_BYTE] & protocol::REPORT_IONIZER1_MASK) != 0;
    bool ionizer2 = (this->serialProcess_.data[protocol::REPORT_IONIZER2_BYTE] & protocol::REPORT_IONIZER2_MASK) != 0;
    return ionizer1 || ionizer2;
}

bool GreeACCNT::determine_beeper(){
    return (this->serialProcess_.data[protocol::REPORT_BEEPER_BYTE] & protocol::REPORT_BEEPER_MASK) == 0;
}

bool GreeACCNT::determine_sleep(){
    return (this->serialProcess_.data[protocol::REPORT_SLEEP_BYTE] & protocol::REPORT_SLEEP_MASK) != 0;
}

bool GreeACCNT::determine_xfan(){
    return (this->serialProcess_.data[protocol::REPORT_XFAN_BYTE] & protocol::REPORT_XFAN_MASK) != 0;
}

bool GreeACCNT::determine_powersave(){
    return (this->serialProcess_.data[protocol::REPORT_POWERSAVE_BYTE] & protocol::REPORT_POWERSAVE_MASK) != 0;
}

bool GreeACCNT::determine_turbo(){
    return (this->serialProcess_.data[protocol::REPORT_FAN_TURBO_BYTE] & protocol::REPORT_FAN_TURBO_MASK) != 0;
}

bool GreeACCNT::determine_ifeel(){
    return (this->serialProcess_.data[protocol::REPORT_IFEEL_BYTE] & protocol::REPORT_IFEEL_MASK) != 0;
}

const char* GreeACCNT::determine_quiet(){
    if (this->serialProcess_.data[protocol::REPORT_FAN_QUIET_BYTE] & protocol::REPORT_FAN_QUIET_MASK)
        return quiet_options::ON;
    if (this->serialProcess_.data[protocol::REPORT_FAN_QUIET_BYTE] & protocol::REPORT_FAN_QUIET_AUTO_MASK)
        return quiet_options::AUTO;
    return quiet_options::OFF;
}


/*
 * Sensor handling
 */

void GreeACCNT::on_vertical_swing_change(const std::string &swing)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting vertical swing position");

    this->mark_for_update_();
    this->vertical_swing_state_ = swing;
}

void GreeACCNT::on_horizontal_swing_change(const std::string &swing)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting horizontal swing position");

    this->mark_for_update_();
    this->horizontal_swing_state_ = swing;
}

void GreeACCNT::on_display_change(const std::string &display)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting display mode");

    this->mark_for_update_();
    this->display_state_ = display;
}

void GreeACCNT::on_display_unit_change(const std::string &display_unit)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting display unit");

    this->mark_for_update_();
    this->display_unit_state_ = display_unit;
}

void GreeACCNT::on_light_mode_change(const std::string &mode)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting light mode to %s", mode.c_str());

    this->mark_for_update_();
    this->light_mode_ = mode;

    if (this->light_mode_ == light_options::AUTO)
    {
        this->light_state_ = (this->mode != climate::CLIMATE_MODE_OFF);
    }
    else if (this->light_mode_ == light_options::ON)
    {
        this->light_state_ = true;
    }
    else
    {
        this->light_state_ = false;
    }
}

void GreeACCNT::on_ionizer_change(bool ionizer)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting ionizer");

    this->mark_for_update_();
    this->ionizer_state_ = ionizer;
}

void GreeACCNT::on_beeper_change(bool beeper)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting beeper");

    this->mark_for_update_();
    this->beeper_state_ = beeper;
}

void GreeACCNT::on_sleep_change(bool sleep)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting sleep");

    this->mark_for_update_();
    this->sleep_state_ = sleep;
}

void GreeACCNT::on_xfan_change(bool xfan)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting xfan");

    this->mark_for_update_();
    this->xfan_state_ = xfan;
}

void GreeACCNT::on_powersave_change(bool powersave)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting powersave");

    this->mark_for_update_();
    this->powersave_state_ = powersave;
}

void GreeACCNT::on_turbo_change(bool turbo)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting turbo");

    this->mark_for_update_();
    this->turbo_state_ = turbo;

    /* Requirement 1: when turbo gets on, quite must get off. */
    if (turbo) {
        this->update_quiet(quiet_options::OFF);
    }
}

void GreeACCNT::on_ifeel_change(bool ifeel)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting ifeel");

    this->mark_for_update_();
    this->ifeel_state_ = ifeel;
}

void GreeACCNT::on_quiet_change(const std::string &quiet)
{
    if (this->state_ != ACState::Ready)
        return;

    ESP_LOGD(TAG, "Setting quiet mode");

    this->mark_for_update_();
    this->quiet_state_ = quiet;

    /* Requirement 1: when gets on/auto then turbo must go off. */
    if (quiet != quiet_options::OFF) {
        this->update_turbo(false);
    }
}

}  // namespace CNT
}  // namespace gree_ac
}  // namespace esphome
