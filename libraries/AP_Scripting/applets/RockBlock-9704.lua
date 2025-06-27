--[[ Lua script to send a recieve very basic MAVLink telemetry over a
Rockblock 9704 SBD satellite modem
Requires https://github.com/stephendade/rockblock2mav at the GCS end

Setup:
This script requires 1 serial port:
A "Script" to connect the RockBlock modem
- Telemetry port connected to pins 13/14 on the Rockblock header
- Telemetry Power  connected to pins 15 (+5V) and 16 (GND) on the Rockblock header
- Servo_out10 (AUXOUT2) (configured as relay) connected to pin 3 (I_EN) on the Rockblock header
- Servo_out11 (configured as GPIO) connected to pin 7 (I_BTD) on the Rockblock header

ftp put ./Documents/UAVCode/ardupilot/libraries/AP_Scripting/applets/RockBlock-9704.lua APM/Scripts/RockBlock-9704.lua

Usage:
Use the MAVLink High Latency Control ("link hl on|off" in MAVProxy) to control
whether to send or not (or use "force_hl_enable")

Written by Stephen Dade (stephen_dade@hotmail.com)
]]--

local port = serial:find_serial(0)

if not port then
    gcs:send_text(3, "Rockblock: No Scripting Serial Port")
    return
end

local EN_RELAY = 0  -- RELAY1 to outout I_EN. Ensure RELAY1_FUNCTION=1 and RELAY1_DEFAULT=0
local I_BTD_GPIO = 52 -- GPIO to output I_BTD. Ensure SERVO11_FUNCTION=-1

gpio:pinMode(I_BTD_GPIO,0) -- set AUX 3 (servo11) to input I_BTD SERIAL11_FUNCTION=-1

port:begin(230400)
port:set_flow_control(0)

local time_last_sent = millis():tofloat() * 0.001

--[[
Lua Object for managing the RockBlock modem
--]]
local function RockblockModem()
    -- public fields
    local self = {
        is_transmitting = false,
        first_sucessful_mailbox_check = false,
        time_last_tx = millis():tofloat() * 0.001,
        modem_state = 0, -- 0=not booted, 1=booting, 2=booted, 3=configured, 4=shutting down
        modem_model = "",
        active = false, -- whether the modem is active or not
        signal_bars = 0, -- signal bars (0-5)
    }

    -- private fields
    local _rxstring = "" -- string to hold received data
    local _waiting_for_response = false -- flag to indicate if we are waiting for a response
    local _modem_to_send = {}
    local _modem_expected_response = {}
    local _topic_provisioned = false -- flag to indicate if the RAW topic is provisioned
    local _req_id = 0 -- request ID for the next message to send
    local _message_id = 0 -- message ID for the current message
    local _next_to_send = nil -- the next message to send

    function self.reset()
        -- reset the modem state
        self.modem_model = "" -- reset the modem model
        self.active = false -- reset the active state
        self.signal_bars = 0 -- reset the signal bars
        self.is_transmitting = false -- reset the transmitting state
        _rxstring = "" -- clear the rxstring
        _waiting_for_response = false -- clear the waiting flag
        _modem_to_send = {} -- clear the modem queue
        _modem_expected_response = {} -- clear the expected response list
    end

    -- Get any incoming data. Returns a tuple of (status, command, data) if we have a full packet
    function self.rxdata(instring)
        -- read = string.char(inchar)
        local finishedPacket = nil
        -- append the read character to the rxstring
        _rxstring = _rxstring .. instring
        -- check if we have a complete message (delimited by \r)
        local end_pos = _rxstring:find('\r')
        if end_pos then
            --we have a complete message, extract it
            local message = _rxstring:sub(1, end_pos - 1)
            -- _rxstring = _rxstring:sub(end_pos + 1) -- remove the message from the buffer
            -- strip out any non-ascii characters
            local message = message:gsub("[^%w%s%p]", "")
            -- split the string according to the format <code>status command {data}</code>
            local status, command, data = message:match("^(%d+)%s(%S+)%s(.*)$")
            if status and command and data then
                gcs:send_text(3, "Rockblock: Parsed response: " .. status .. " " .. command .. " " .. data)
                _rxstring = _rxstring:sub(end_pos + 1) -- remove the message from the buffer
                return tonumber(status), command, data
            end
        end
        return nil, nil, nil
    end

    function self.ready_to_send()
        -- check if the modem is ready to send a message
        if self.modem_state ~= 3 then
            return false
        end
        if not self.active then
            return false
        end
        if not _topic_provisioned then
            return false
        end
        if _next_to_send then
            return false
        end
        return true -- ready to send a message
    end

    function self.send_message(mavlink_msg_bytes)
        -- send a MAVLink message over the RockBlock modem
        if not mavlink_msg_bytes or #mavlink_msg_bytes == 0 then
            gcs:send_text(3, "Rockblock: No message to send")
            return
        end
        -- check the correct topic is active
        if not _topic_provisioned then
            gcs:send_text(3, "Rockblock: Topic not privisioned: " .. _topic)
            return
        end
        -- check we're not already sending a message
        if _next_to_send then
            gcs:send_text(3, "Rockblock: Already sending a message")
            return
        end
        -- send a messageOriginate
        _req_id = _req_id + 1 -- increment the request ID
        _message_id = 0 -- reset the message ID
        -- store the message to send, with CRC and encoding TODO
        _next_to_send = mavlink_msg_bytes
        self.send_command("PUT", "messageOriginate", "\"topic_id\": 244, \"message_length\":" .. #mavlink_msg_bytes-5 .. ", \"request_reference\":" .. _req_id .. "")
    end

    function self.send_command(getput, cmd, options)
        -- add command to the modem queue
        if not cmd or cmd == "" then
            gcs:send_text(3, "Rockblock: No command to send")
            return
        end
        -- limit command queue to 5 commands
        if #_modem_to_send >= 5 then
            gcs:send_text(3, "Rockblock: Command queue full, dropping command: " .. cmd)
            return
        end
        -- stop repeat commands if already waiting for a response
        if _waiting_for_response and cmd == _waiting_for_response then
            return
        end
        local full_cmd = getput .. " " .. cmd .. " {" .. options .. "}" .. "\r" -- append \r to the end of the command
        -- don't add if the last command is the same as the new command
        if #_modem_to_send > 0 and _modem_to_send[#_modem_to_send] == full_cmd then
            return
        end
        _modem_to_send[#_modem_to_send + 1] = full_cmd
        _modem_expected_response[#_modem_expected_response + 1] = cmd -- add the command to the expected response list
    end

    function self.process_modem_periodic(instring)
        -- send the next command in the modem queue if we're not waiting for a response
        if not _waiting_for_response and #_modem_to_send > 0 then
            local full_cmd = table.remove(_modem_to_send, 1)
            gcs:send_text(3, "Rockblock: Sending command: " .. full_cmd)
            for idx = 1, #full_cmd do
                port:write(full_cmd:byte(idx))
            end
            _waiting_for_response = table.remove(_modem_expected_response, 1) -- get the expected response command
            gcs:send_text(3, "Rockblock: Waiting for response to command: " .. _waiting_for_response)
            self.time_last_tx = millis():tofloat() * 0.001 -- update last time
        end
        -- timeout for waiting for a response
        if _waiting_for_response and (millis():tofloat() * 0.001) - self.time_last_tx > 5 then
            -- if we haven't received a response in 5 seconds, then timeout
            gcs:send_text(3, "Rockblock: Command timeout: " .. _waiting_for_response)
            _waiting_for_response = false -- clear the waiting flag
        end
        -- check if we have a response from the modem
        if instring ~= "" then
            status, command, data = self.rxdata(instring)
            if status and command then
                if (status == 200 or status == 402 or status == 299) then
                    -- if we get a 200 response, then the command succeeded
                    -- 402 response is configuration already set
                    -- 299 response is an unsolicited response
                    if command == _waiting_for_response then
                        _waiting_for_response = false -- clear the waiting flag
                    end
                    gcs:send_text(3, "Rockblock: Command success: " .. command)
                    -- if it's a responce to the PUT API message then change modem state
                    if command == "apiVersion" then
                        self.modem_state = 3 -- modem is configured
                        gcs:send_text(3, "Rockblock: Modem configured")
                        -- clear the modem queue
                        _modem_to_send = {}
                        _modem_expected_response = {}
                    elseif command == "hwInfo" then
                        -- if we get the hardware info, then set the modem model
                        -- {"hw_version":"0x0601","serial_number":"1a0236","imei":"300258060104610","board_temp":12}
                        self.modem_model = data:match('"hw_version":"(.-)"')
                        gcs:send_text(3, "Rockblock: hwversion: " .. self.modem_model)
                    elseif command == "operationalState" then
                        -- if we get the operational state, then set the modem state
                        -- operationalState {"state":"inactive","reason":0}
                        local modem_state_response = data:match('"state":"(.-)"')
                        if modem_state_response == "inactive" then
                            self.active = false
                        elseif modem_state_response == "active" then
                            self.active = true -- modem is active
                        end
                        gcs:send_text(3, "Rockblock: operational state: " .. tostring(self.active))
                    elseif command == "constellationState" then
                        -- if we get the constellation state, then set the signal bars
                        -- {"signal_bars":5,"signal_strength":-60}
                        self.active = true -- modem is active by definition. Stops spamming of PUT operationalState
                        local signal_bars = data:match('"signal_bars":(%d+)')
                        if signal_bars then
                            self.signal_bars = tonumber(signal_bars)
                            gcs:send_text(3, "Rockblock: Signal bars: " .. tostring(self.signal_bars) .. "/5")
                        end
                    elseif command == "messageProvisioning" then
                        -- if we get the message provisioning, then check if the topic is provisioned
                        -- {"provisioning":[{"topic_id":244,"topic_name":"BAD","priority":"High","discard_time_seconds":604800,"max_queue_depth":99}]
                        local topic_provisioned = data:match('"topic_id":%d+,"topic_name":"RAW"')
                        if topic_provisioned then
                            _topic_provisioned = true -- topic is provisioned
                            gcs:send_text(3, "Rockblock: Modem is provisioned")
                        else
                            _topic_provisioned = false -- topic is not provisioned
                            gcs:send_text(3, "Rockblock: Error: Modem is not provisioned")
                        end
                    elseif command == "messageOriginate" then
                        -- Rockblock is ready to accept a message
                        -- {"topic_id":244,"request_reference":1,"message_id":1,"message_response":"message_accepted"}
                        gcs:send_text(3, "Rockblock: Message Originate response: " .. data)
                        -- get the message ID from the response and confirm accepted and the request reference matches
                        local request_reference = data:match('"request_reference":(%d+)')
                        local accepted = data:match('"message_response":"(.-)"')
                        if tonumber(request_reference) == _req_id and accepted == "message_accepted" then
                            _message_id = tonumber(data:match('"message_id":(%d+)'))
                            -- send the message
                            -- PUT messageOriginateSegment {"topic_id":244, "message_id":2, "segment_length":15, "segment_start":0, "data":"SGVsbG8sIHdvcmxkIXre"}
                            local segment = "\"topic_id\":244, \"message_id\":" .. _message_id .. ", \"segment_length\":" .. #_next_to_send-5 .. ", \"segment_start\":0, \"data\":\"" .. _next_to_send .. "\""
                            -- gcs:send_text(3, "Rockblock: Sending message segment: " .. segment)
                            self.send_command("PUT", "messageOriginateSegment", segment)
                        end
                    elseif command == "messageOriginateStatus" then
                        -- message send status
                        -- {"topic_id":244,"message_id":1,"final_mo_status":"mo_ack_received"}
                        gcs:send_text(3, "Rockblock: Message Originate Status response: " .. data)
                        -- if the final status is mo_ack_received for the message_id, then the message was sent successfully
                        local final_status = data:match('"final_mo_status":"(.-)"')
                        local message_id = data:match('"message_id":(%d+)')
                        if final_status == "mo_ack_received" and tonumber(message_id) == _message_id then
                            gcs:send_text(3, "Rockblock: Message sent successfully with ID " .. _message_id)
                            _next_to_send = nil -- clear the next message to send
                        else
                            gcs:send_text(3, "Rockblock: Message send failed with ID " .. _message_id .. " and status " .. final_status)
                            _next_to_send = nil -- clear the next message to send
                        end
                    end

                elseif command == _waiting_for_response then
                    -- if we get a non-good response, then the command failed
                    _waiting_for_response = false -- clear the waiting flag
                    gcs:send_text(3, "Rockblock: Command failed: " .. command)
                end
            end
        end
        -- if there's more than 3 commands in the queue, then don't go down to the next step
        if #_modem_to_send > 3 then
            return
        end
        -- to prevent flooding the modem with commands, we wait 3 seconds before sending the next command for initialisation
        if (millis():tofloat() * 0.001) - self.time_last_tx > 3 then
            --initialise the modem (try once per 3 sec)
            if self.modem_state == 2 then
                gcs:send_text(3, "Rockblock: Configuring modem")
                self.send_command("PUT", "apiVersion", "\"active_version\": {\"major\": 1, \"minor\": 6, \"patch\": 1}") -- send the command to set the API version
                return
            end
            -- if powered, query the modem for model number and initial state. Also set the SIM state to internal
            if self.modem_state == 3 and self.modem_model == "" then
                gcs:send_text(3, "Rockblock: Querying modem for model number")
                self.send_command("GET", "hwInfo", "") -- send the command to get the hardware info
                self.send_command("PUT", "simConfig", "\"interface\": \"internal\"")
                --self.send_command("GET", "operationalState", "") -- send the command to get the operational state
                return
            end
            -- if the modem is not active, then set it to active
            if self.modem_state == 3 and not self.active then
                gcs:send_text(3, "Rockblock: Setting modem to active")
                self.send_command("PUT", "operationalState", "\"state\": \"active\"") -- send the command to set the operational state to active
                return
            end
            -- if the modem is active, check for topic provisioning
            if self.modem_state == 3 and self.active and not _topic_provisioned then
                gcs:send_text(3, "Rockblock: Checking topic provisioning")
                self.send_command("GET", "messageProvisioning", "") -- send the command to get the message provisioning
                return
            end
        end
    end

    -- return the instance
    return self
end

-- Define the RockBlock interface
local rockblock = RockblockModem()

function HLSatcom()
    -- boot the modem
    if gcs:get_high_latency_status() and rockblock.modem_state == 0 then
        -- set I_EN to high to power the modem, if not already powered
        relay:on(EN_RELAY) -- turn on I_EN
        gcs:send_text(3, "Rockblock: Powering modem")
        rockblock.modem_state = 1
    end
    if rockblock.modem_state == 1 and gpio:read(I_BTD_GPIO) then
        rockblock.modem_state = 2 -- modem is booted
        gcs:send_text(3, "Rockblock: Modem booted")
        -- reset the modem state
        rockblock.reset()
        -- drain the serial port rx buffer
        local n_bytes = port:available()
        while n_bytes > 0 do
            n_bytes = n_bytes - 1
            port:read()
        end
    end

    -- shut down the modem
    if not gcs:get_high_latency_status() and (rockblock.modem_state == 3 or rockblock.modem_state == 2 or rockblock.modem_state == 1) then
        -- set I_EN to low to power off the modem
        relay:off(EN_RELAY) -- turn off I_EN
        gcs:send_text(3, "Rockblock: Powering down modem")
        rockblock.modem_state = 4 -- modem is shutting down
    end
    if rockblock.modem_state == 4 and not gpio:read(I_BTD_GPIO) then
        rockblock.modem_state = 0 -- modem is off
        gcs:send_text(3, "Rockblock: Modem Shutdown")
    end

    -- waiting states
    if gcs:get_high_latency_status() and rockblock.modem_state == 1 then
        -- wait for the modem to boot
        gcs:send_text(3, "Rockblock: Waiting for modem to boot")
        return protected_wrapper, 5000 -- wait 5 seconds before checking again
    end
    if not gcs:get_high_latency_status() and (rockblock.modem_state == 3 or rockblock.modem_state == 4) then
        gcs:send_text(3, "Rockblock: Waiting for modem to power off")
        return protected_wrapper, 5000 -- wait 5 seconds before checking again
    end

    -- if powered, check for incoming data
    if rockblock.modem_state == 2 or rockblock.modem_state == 3 then
        local n_bytes = math.min(port:available(), 50)
        local rxstring = "" -- string to hold received data
        while n_bytes > 0 do
            read = port:read()
            n_bytes = n_bytes - 1
            rxstring = rxstring .. string.char(read)
        end
        rockblock.process_modem_periodic(rxstring) -- process the incoming data
    end

    --if active, send a message once per 30 seconds
    if gcs:get_high_latency_status() and rockblock.ready_to_send() and (millis():tofloat() * 0.001) - time_last_sent > 30 then
        rockblock.send_message("SGVsbG8sIHdvcmxkIXre") -- Hello World!
        gcs:send_text(3, "Rockblock: Sending message")
        time_last_sent = millis():tofloat() * 0.001 -- update last time sent
    end
end

-- wrapper around HLSatcom(). This calls HLSatcom() and if HLSatcom faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
    local success, err = pcall(HLSatcom)
    if not success then
        gcs:send_text(3, "Internal Error: " .. err)
        -- when we fault we run the HLSatcom function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, math.floor(1000 / 10) -- run every 100ms
end

-- start running HLSatcom loop
return protected_wrapper()
