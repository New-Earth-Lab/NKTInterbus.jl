using LibSerialPort
using CRC

import Base.push!

# Define enumeration for RxStates
@enum RxStates Hunting_SOT Hunting_EOT Port_Lost Timeout_Error Message_Ready Content_Error CRC_Error Garbage_Error Overrun_Error

# Define message types
@enum MsgType begin
    MSG_NACK = 0x00
    MSG_CRC = 0x01
    MSG_BUSY = 0x02
    MSG_ACK = 0x03
    MSG_RDCMD = 0x04
    MSG_WRCMD = 0x05
    MSG_WRSET = 0x06
    MSG_WRCLR = 0x07
    MSG_DATAGRAM = 0x08
    MSG_WRTGL = 0x09
    MSG_NONE = 0xFF
end

# Constants for Interbus message structure
const SOT = 0x0D
const EOT = 0x0A
const SOE = 0x5e
const ECC = 0x40

# Define the struct to hold the InterbusFunc data
mutable struct Interbus <: Core.IO
    sp::SerialPort
    master_id::UInt8
    rx_state::RxStates
    rx_escape::Bool
    crc_func
    tx_buffer::Vector{UInt8}
    rx_buffer::Vector{UInt8}
    Interbus(sp, master_id) = new(sp, master_id, Hunting_SOT, false, crc(CRC_16_XMODEM), UInt8[], UInt8[])
end

function push!(bus::Interbus, data::Vector{UInt8})
    for d in data
        push!(bus, d)
    end
end

function push!(bus::Interbus, data::UInt8)
    if data in [SOT, EOT, SOE]
        push!(bus.tx_buffer, SOE)
        data += ECC
    end
    push!(bus.tx_buffer, data)
end

function flush!(bus::Interbus)
    sp_flush(bus.sp, SP_BUF_INPUT)
end

function send!(bus::Interbus, device_id::UInt8, register_id::UInt8, msgType::MsgType, data::Vector{UInt8})
    empty!(bus.tx_buffer)
    push!(bus.tx_buffer, SOT) # Start of Telegram
    push!(bus, device_id)
    push!(bus, bus.master_id)
    push!(bus, UInt8(msgType))
    push!(bus, register_id)
    if msgType in [MSG_WRCMD MSG_WRCLR MSG_WRSET MSG_WRTGL]
        push!(bus, data)  # Add write data to message
    end
    crc = bus.crc_func(bus.tx_buffer[2:end])
    push!(bus, UInt8(crc >> 8))
    push!(bus, UInt8(crc & 0xff))
    push!(bus.tx_buffer, EOT) # End of Telegram
    write(bus.sp, bus.tx_buffer)
end

function receive!(bus::Interbus, device_id::UInt8, register_id::UInt8, msgType::UInt8)
    payload = UInt8[]
    rxCh::UInt8 = 0
    bus.rx_state = Hunting_SOT
    while bus.rx_state in [Hunting_SOT, Hunting_EOT]
        try
            rxCh = read(bus.sp, UInt8)
        catch error
            if isa(error, LibSerialPort.Timeout)
                bus.rx_state = Timeout_Error
            else
                bus.rx_state = Port_Lost
            end
        end
        if bus.rx_state == Hunting_SOT      # Hunting Start Of Telegram
            if rxCh == SOT
                # Got start of telegram
                empty!(bus.rx_buffer)
                bus.rx_escape = false
                bus.rx_state = Hunting_EOT
            end
        elseif bus.rx_state == Hunting_EOT   # Hunting End Of Telegram while collecting telegram
            if rxCh == EOT
                # Got end of telegram
                if length(bus.rx_buffer) >= 5     # host_id + device_id + msgType + CRCH + CRCL - Minimum telegram length
                    if bus.crc_func(bus.rx_buffer) == 0
                        # We have collected a message with valid CRC - Check the contents
                        if bus.rx_buffer[2] == device_id && bus.rx_buffer[3] == msgType && bus.rx_buffer[4] == register_id
                            resize!(payload, length(bus.rx_buffer) - 5)
                            payload = copy(bus.rx_buffer[5:end-2])
                            bus.rx_state = Message_Ready
                        else
                            bus.rx_state = Content_Error
                        end
                    else
                        bus.rx_state = CRC_Error
                    end
                else
                    bus.rx_state = Garbage_Error
                end
            else
                # Collecting telegram
                if rxCh == SOE
                    # Got escape sequence
                    bus.rx_escape = true
                else
                    # Got normal telegram contents
                    if bus.rx_escape
                        rxCh -= ECC
                        bus.rx_escape = false
                    end
                    push!(bus.rx_buffer, rxCh)
                end
            end
        end
    end
    return payload, bus.rx_state
end

function Base.open(portname::AbstractString, baudrate::Integer, master_id::UInt8, timeout::Real=0.01)
    sp = LibSerialPort.open(portname, baudrate)
    set_read_timeout(sp, timeout)
    set_write_timeout(sp, timeout)
    return Interbus(sp, master_id)
end

function Base.open(f::Function, portname::AbstractString, baudrate::Integer, master_id::UInt8, timeout::Real=0.01)
    bus = open(portname, baudrate, master_id, timeout)
    try
        f(bus)
    finally
        close(bus)
    end
end

function Base.close(bus::Interbus)
    if isopen(bus)
        close(bus.sp)
    end
end

Base.isopen(bus::Interbus) = isopen(bus.sp)

function Base.read(bus::Interbus, device_id, register_id)
    if isopen(bus)
        flush!(bus)
        send!(bus, device_id, register_id, MSG_RDCMD, UInt8[])
        payload, state = receive!(bus, device_id, register_id, UInt8(MSG_DATAGRAM))
        if state == Message_Ready
            return payload
        end
    end
end

function Base.write(bus::Interbus, device_id, register_id, x)
    if isopen(bus)
        flush!(bus)
        send!(bus, device_id, register_id, MSG_WRCMD, reinterpret(UInt8, x))
        _, state = receive!(bus, device_id, register_id, UInt8(MSG_ACK))
        if state == Message_Ready
            return true
        end
    end
    return false
end

function Base.write(bus::Interbus, device_id, register_id, x)
    if isopen(bus)
        flush!(bus)
        send!(bus, device_id, register_id, MSG_WRCMD, reinterpret(UInt8, x))
        _, state = receive!(bus, device_id, register_id, UInt8(MSG_ACK))
        if state == Message_Ready
            return true
        end
    end
    return false
end